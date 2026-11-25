#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace wdf::android::usb {

struct Format {
    unsigned sampleRate = 0;
    unsigned inputChannels = 0;
    unsigned outputChannels = 0;
};

struct Statistics {
    std::uint64_t completedFrames = 0;
    std::uint64_t inputOverruns = 0;
    std::uint64_t inputUnderruns = 0;
    std::uint64_t packetErrors = 0;
    std::uint64_t captureRecoveries = 0;
};

using AudioCallback = void (*)(void* context, const float* inputInterleaved,
                               float* outputInterleaved, unsigned frames) noexcept;

// Direct USB audio using an fd obtained with Android's USB Host permission.
// Call lifecycle methods from one control thread. The callback is invoked on
// a worker and must not block, allocate, or call a lifecycle method. Input is
// null for playback-only devices. Both arrays are valid only until the callback
// returns. The output buffer is zeroed before every callback.
class Device {
public:
    Device();
    ~Device();

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    // Duplicates fd. Keep UsbDeviceConnection alive through close() and avoid
    // other USB requests on it while this device owns the stream interfaces.
    bool open(int fd, std::span<const std::uint8_t> rawDescriptors, unsigned sampleRate = 48000);
    bool start(unsigned blockFrames, AudioCallback callback, void* context);
    // Waits for all callbacks and USB transfers to finish. It is safe to start
    // again after stop(). close() also restores and releases the interfaces.
    void stop() noexcept;
    void close() noexcept;

    Format format() const noexcept;
    bool isRunning() const noexcept;
    Statistics statistics() const noexcept;
    std::string lastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace wdf::android::usb
