/*
 * SPDX-FileCopyrightText: 2017-2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "ConsumerIr"

#include <fcntl.h>
#include <linux/lirc.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <ir.sysprop.h>

#include "ConsumerIr.h"

using namespace ::vendor::lineage::ir;

namespace aidl {
namespace android {
namespace hardware {
namespace ir {

/*
 * MediaTek boards that boot with CONFIG_MTK_IRTX_PWM_SUPPORT instead of
 * CONFIG_LIRC do not expose /dev/lirc0 at all. Their kernel driver speaks a
 * completely different protocol, so the LIRC path can never work there.
 *
 * Mirrored from drivers/misc/mediatek/irtx/mtk_irtx.h, which is not exported
 * to userspace.
 */
#define IRTX_IOC_SET_CARRIER_FREQ _IOW('R', 0, unsigned int)

static const std::string kLircDevice = "/dev/lirc0";
static const std::string kIrtxDevice = "/dev/irtx";

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
 * The IRTX driver's write() takes a bitstream rather than LIRC durations:
 * every bit covers one carrier period (1 = carrier on, 0 = off), packed LSB
 * first into 32 bit words. The last word holds the total duration in
 * microseconds, while the driver only hands (size / 4) - 2 words to the PWM
 * hardware, hence the trailing spare words.
 */
static ::ndk::ScopedAStatus transmitIrtx(int fd, int32_t carrierFreqHz,
                                         const std::vector<int32_t>& pattern) {
    int rc = ioctl(fd, IRTX_IOC_SET_CARRIER_FREQ, &carrierFreqHz);
    if (rc < 0) {
        LOG(ERROR) << "Failed to set carrier " << carrierFreqHz << ", error: " << errno;
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    uint64_t bitCount = 0;
    uint64_t totalUs = 0;
    std::vector<uint32_t> periods(pattern.size());

    for (size_t i = 0; i < pattern.size(); i++) {
        const uint32_t durationUs = pattern[i] > 0 ? pattern[i] : 0;
        const uint64_t entryPeriods =
                (static_cast<uint64_t>(durationUs) * carrierFreqHz + 500000) / 1000000;

        periods[i] = static_cast<uint32_t>(entryPeriods);
        bitCount += entryPeriods;
        totalUs += durationUs;
    }

    const size_t words = (bitCount + 31) / 32 + 3;
    std::vector<uint32_t> wave(words, 0);

    size_t bit = 0;
    bool carrierOn = true;
    for (size_t i = 0; i < periods.size(); i++) {
        for (uint32_t n = 0; n < periods[i]; n++, bit++) {
            if (carrierOn) {
                wave[bit / 32] |= 1u << (bit % 32);
            }
        }
        carrierOn = !carrierOn;
    }
    wave[words - 1] = static_cast<uint32_t>(totalUs);

    rc = write(fd, wave.data(), words * sizeof(uint32_t));
    if (rc < 0) {
        LOG(ERROR) << "Failed to write pattern, " << words << " words, error: " << errno;
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    return ::ndk::ScopedAStatus::ok();
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

    ::android::base::unique_fd fd(open(kIrtxDevice.c_str(), O_WRONLY));
    if (fd.ok()) {
        return transmitIrtx(fd, carrierFreqHz, pattern);
    }

    if (errno != ENOENT) {
        LOG(ERROR) << "Failed to open " << kIrtxDevice << ", error: " << strerror(errno);
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    fd.reset(open(kLircDevice.c_str(), O_WRONLY));
    if (!fd.ok()) {
        LOG(ERROR) << "Failed to open " << kLircDevice << ", error: " << strerror(errno);
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    int rc = ioctl(fd, LIRC_SET_SEND_CARRIER, &carrierFreqHz);
    if (rc < 0) {
        LOG(ERROR) << "Failed to set carrier " << carrierFreqHz << ", error: " << errno;
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    /*
     * Pattern is an alternating series of on and off periods.
     * Kernel needs pattern to have an odd size, which means ending with
     * an on period. If pattern is even in size, drop the last off period.
     */
    if (entries % 2 == 0) {
        entries--;
    }

    rc = write(fd, pattern.data(), entries * sizeof(int32_t));
    if (rc < 0) {
        LOG(ERROR) << "Failed to write pattern, " << entries << " entries, error: " << errno;
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    return ::ndk::ScopedAStatus::ok();
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
