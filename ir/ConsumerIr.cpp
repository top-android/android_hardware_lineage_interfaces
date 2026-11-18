/*
 * SPDX-FileCopyrightText: 2017-2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "ConsumerIr"

#include <fcntl.h>
#include <linux/lirc.h>
#include <sys/ioctl.h>

#include <cstdint>
#include <string>
#include <vector>

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <hardware/hardware.h>
#include <ir.sysprop.h>

#include "ConsumerIr.h"

using namespace ::vendor::lineage::ir;

// cannon: vendor 里带 MTK 官方的 consumerir HAL（/vendor/lib64/hw/consumerir.common.so），
// ro.hardware.consumerir=common 正好让 hw_get_module 找到它。它内部处理了
// vendor.irtx.hal.mode / duty、buffer 格式、IRTX+PWM 方案选择，比手写协议可靠。
// 这里直接复用，自写实现只作为兜底。
typedef struct consumerir_freq_range {
    int min;
    int max;
} consumerir_freq_range_t;

typedef struct consumerir_device {
    struct hw_device_t common;
    int (*transmit)(struct consumerir_device* dev, int carrier_freq, const int pattern[],
                    int pattern_len);
    int (*get_num_carrier_freqs)(struct consumerir_device* dev);
    int (*get_carrier_freqs)(struct consumerir_device* dev, size_t len,
                             consumerir_freq_range_t* ranges);
    void* reserved[5];
} consumerir_device_t;

namespace {

consumerir_device_t* getMtkHal() {
    static consumerir_device_t* sDev = []() -> consumerir_device_t* {
        const hw_module_t* module = nullptr;
        int rc = hw_get_module("consumerir", &module);
        if (rc != 0 || module == nullptr) {
            LOG(WARNING) << "IRTX: hw_get_module(consumerir) failed: " << rc;
            return nullptr;
        }
        hw_device_t* dev = nullptr;
        rc = module->methods->open(module, "transmitter", &dev);
        if (rc != 0 || dev == nullptr) {
            LOG(WARNING) << "IRTX: consumerir open failed: " << rc;
            return nullptr;
        }
        LOG(INFO) << "IRTX: using MTK consumerir HAL (consumerir.common.so)";
        return reinterpret_cast<consumerir_device_t*>(dev);
    }();
    return sDev;
}

}  // namespace

namespace aidl {
namespace android {
namespace hardware {
namespace ir {

// cannon: 本机 IR 节点是 /dev/irtx（MTK IRTX 驱动），没有 /dev/lirc*
static const std::string kIrDevice = "/dev/irtx";
static const std::string kLircDevice = "/dev/lirc0";

ConsumerIr::ConsumerIr() {
    auto carrier_freqs = IrProperties::carrier_freqs();

    if (carrier_freqs.size() >= 2) {
        for (size_t i = 0; i < carrier_freqs.size() - 1; i += 2) {
            if (!carrier_freqs[i] || !carrier_freqs[i + 1]) {
                continue;
            }

            kRangeVec.push_back({
                    .minHz = carrier_freqs[i].value(),
                    .maxHz = carrier_freqs[i + 1].value(),
            });
        }
    } else {
        kRangeVec.push_back({.minHz = 30000, .maxHz = 60000});
    }
}

::ndk::ScopedAStatus ConsumerIr::getCarrierFreqs(std::vector<ConsumerIrFreqRange>* _aidl_return) {
    *_aidl_return = kRangeVec;

    return ::ndk::ScopedAStatus::ok();
}

/*
 * MTK IRTX driver (drivers/misc/mediatek/irtx) does not implement the LIRC
 * ioctls, it only knows IRTX_IOC_SET_CARRIER_FREQ plus a private write() buffer.
 * Buffer layout (from the vendor HAL and mtk_irtx_pwm.c):
 *   words = ceil(sum(cycles) / 32)
 *   [0 .. words-1]  bitmap, one bit per carrier cycle, 1 = LED on, first entry on
 *   [words]         0
 *   [words + 1]     0
 *   [words + 2]     total pattern duration in us (driver usleeps on it)
 * Driver sends BUF0_SIZE = count/4 - 2 words to the PWM hardware.
 */
#define IRTX_IOC_SET_CARRIER_FREQ _IOW('R', 0, unsigned int)
#define IRTX_IOC_SET_DUTY_CYCLE _IOW('R', 2, unsigned int)

static bool transmitMtk(int fd, int32_t carrierFreqHz, const std::vector<int32_t>& pattern) {
    if (ioctl(fd, IRTX_IOC_SET_CARRIER_FREQ, &carrierFreqHz) < 0) {
        LOG(ERROR) << "IRTX: failed to set carrier " << carrierFreqHz << ", error: " << errno;
        return false;
    }

    // cannon: 驱动里 h_l_period = 26MHz * duty / (carrier * cycle)，PWM 周期
    // = HDURATION + LDURATION = 2 * h_l_period。只有 duty/cycle = 1/2 时
    // 2 * h_l_period 才等于 26MHz/carrier（请求的载频本身）。
    // 不设的话驱动默认 cycle=3、duty=1，载频会变成 1.5 倍（38k 请求实际
    // 输出 57 kHz），且 buffer 每个 bit 的时间也跟着缩水 1/3。
    // 对应 dmesg：不设时 period:228（错），设完应为 period:342（26MHz/38k/2）。
    unsigned int duty = (1u << 16) | 2u; // duty_cycle=1, cycle=2 → 50%
    if (ioctl(fd, IRTX_IOC_SET_DUTY_CYCLE, &duty) < 0) {
        LOG(ERROR) << "IRTX: failed to set duty cycle, error: " << errno;
        return false;
    }

    std::vector<uint32_t> cycles(pattern.size());
    uint64_t totalBits = 0;
    uint64_t totalUs = 0;
    for (size_t i = 0; i < pattern.size(); i++) {
        int64_t us = pattern[i] > 0 ? pattern[i] : 0;
        cycles[i] = static_cast<uint32_t>((us * carrierFreqHz + 500000) / 1000000);
        totalBits += cycles[i];
        totalUs += us;
    }

    size_t words = static_cast<size_t>((totalBits + 31) / 32);
    std::vector<uint32_t> buf(words + 3, 0);

    size_t bit = 0;
    bool on = true;
    for (size_t i = 0; i < pattern.size(); i++) {
        if (on) {
            for (uint32_t c = 0; c < cycles[i]; c++, bit++) {
                buf[bit >> 5] |= 1u << (bit & 31);
            }
        } else {
            bit += cycles[i];
        }
        on = !on;
    }
    buf[words + 2] = static_cast<uint32_t>(totalUs);

    // cannon: 便于对齐波形，打印载频、总时长和前若干段原始 pattern（us）
    {
        std::string head;
        size_t n = pattern.size() < 24 ? pattern.size() : 24;
        for (size_t i = 0; i < n; i++) {
            head += std::to_string(pattern[i]);
            head += (i + 1 < n) ? "," : "";
        }
        LOG(INFO) << "IRTX tx: carrier=" << carrierFreqHz << " entries=" << pattern.size()
                  << " totalUs=" << totalUs << " bits=" << totalBits << " head=[" << head << "]";
    }

    ssize_t written = write(fd, buf.data(), buf.size() * sizeof(uint32_t));
    if (written != static_cast<ssize_t>(buf.size() * sizeof(uint32_t))) {
        LOG(ERROR) << "IRTX: write failed, size=" << buf.size() * sizeof(uint32_t)
                   << ", error: " << errno;
        return false;
    }
    return true;
}

static bool transmitLirc(int fd, int32_t carrierFreqHz, const std::vector<int32_t>& pattern,
                         size_t entries) {
    if (ioctl(fd, LIRC_SET_SEND_CARRIER, &carrierFreqHz) < 0) {
        LOG(ERROR) << "Failed to set carrier " << carrierFreqHz << ", error: " << errno;
        return false;
    }

    /*
     * Pattern is an alternating series of on and off periods.
     * Kernel needs pattern to have an odd size, which means ending with
     * an on period. If pattern is even in size, drop the last off period.
     */
    if (entries % 2 == 0) {
        entries--;
    }

    if (write(fd, pattern.data(), entries * sizeof(int32_t)) < 0) {
        LOG(ERROR) << "Failed to write pattern, " << entries << " entries, error: " << errno;
        return false;
    }
    return true;
}

::ndk::ScopedAStatus ConsumerIr::transmit(int32_t carrierFreqHz,
                                          const std::vector<int32_t>& pattern) {
    size_t entries = pattern.size();

    if (entries == 0) {
        return ::ndk::ScopedAStatus::ok();
    }

    if (!isInRange(carrierFreqHz)) {
        LOG(ERROR) << "Unsupported carrier " << carrierFreqHz;
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    // cannon: 优先走 MTK 官方 consumerir HAL
    if (consumerir_device_t* dev = getMtkHal()) {
        int rc = dev->transmit(dev, carrierFreqHz, pattern.data(),
                               static_cast<int>(pattern.size()));
        if (rc == 0) {
            return ::ndk::ScopedAStatus::ok();
        }
        LOG(WARNING) << "IRTX: MTK HAL transmit rc=" << rc << ", falling back";
    }

    ::android::base::unique_fd fd(open(kIrDevice.c_str(), O_WRONLY));
    if (fd.ok()) {
        return transmitMtk(fd.get(), carrierFreqHz, pattern)
                       ? ::ndk::ScopedAStatus::ok()
                       : ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    LOG(WARNING) << "Failed to open " << kIrDevice << " (" << strerror(errno)
                 << "), falling back to " << kLircDevice;
    ::android::base::unique_fd lirc(open(kLircDevice.c_str(), O_WRONLY));
    if (!lirc.ok()) {
        LOG(ERROR) << "Failed to open " << kLircDevice << ", error: " << strerror(errno);
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    return transmitLirc(lirc.get(), carrierFreqHz, pattern, entries)
                   ? ::ndk::ScopedAStatus::ok()
                   : ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
}

bool ConsumerIr::isInRange(int32_t carrierFreqHz) {
    for (const auto& range : kRangeVec) {
        if (carrierFreqHz >= range.minHz && carrierFreqHz <= range.maxHz) {
            return true;
        }
    }
    return false;
}

}  // namespace ir
}  // namespace hardware
}  // namespace android
}  // namespace aidl
