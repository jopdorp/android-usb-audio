#include "UsbAudioDescriptors.h"
#include "UsbAudioDevice.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <string_view>
#include <unistd.h>
#include <vector>

#if defined(ANDROID_USB_AUDIO_TEST_USBFS)
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <deque>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <thread>
#endif

using namespace wdf::android::usb;
using namespace wdf::android::usb::detail;

namespace {

using Config = std::vector<std::uint8_t>;

void check(bool condition, std::string_view description) {
    if (!condition) {
        std::cerr << description << '\n';
        std::exit(1);
    }
}

void append(Config& config, std::initializer_list<std::uint8_t> descriptor) {
    check(descriptor.size() == *descriptor.begin(), "Invalid test descriptor length");
    config.insert(config.end(), descriptor);
    config[2] = static_cast<std::uint8_t>(config.size());
    config[3] = static_cast<std::uint8_t>(config.size() >> 8u);
}

Config uac1() {
    Config config{9, 2, 9, 0, 3, 1, 0, 0x80, 50};
    append(config, {9, 4, 0, 0, 0, 1, 1, 0, 0});
    append(config, {10, 0x24, 1, 0, 1, 10, 0, 2, 1, 2});
    append(config, {9, 4, 1, 0, 0, 1, 2, 0, 0});
    append(config, {9, 4, 1, 1, 1, 1, 2, 0, 0});
    append(config, {7, 0x24, 1, 1, 1, 1, 0});
    append(config, {11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0});
    append(config, {9, 5, 0x01, 9, 192, 0, 1, 0, 0});
    append(config, {7, 0x25, 1, 0, 0, 0, 0});
    append(config, {9, 4, 2, 0, 0, 1, 2, 0, 0});
    append(config, {9, 4, 2, 1, 1, 1, 2, 0, 0});
    append(config, {7, 0x24, 1, 5, 1, 1, 0});
    append(config, {14, 0x24, 2, 1, 1, 2, 16, 0, 0x44, 0xac, 0, 0x80, 0xbb, 0});
    append(config, {9, 5, 0x82, 5, 98, 0, 1, 0, 0});
    append(config, {7, 0x25, 1, 1, 0, 0, 0});
    return config;
}

Config uac2(bool feedback, bool capture = true) {
    Config config{9, 2, 9, 0, 3, 2, 0, 0x80, 50};
    append(config, {8, 11, 4, 3, 1, 0, 0x20, 0});
    append(config, {9, 4, 4, 0, 0, 1, 1, 0x20, 0});
    append(config, {9, 0x24, 1, 0, 2, 1, 0, 0, 0});
    append(config, {8, 0x24, 10, 0x21, 3, 3, 0, 0});
    append(config, {8, 0x24, 11, 0x22, 1, 0x21, 1, 0});
    append(config, {17, 0x24, 2, 1, 1, 1, 0, 0x22, 2, 3, 0, 0, 0, 0, 0, 0, 0});
    append(config, {12, 0x24, 3, 2, 1, 1, 0, 1, 0x21, 0, 0, 0});
    append(config, {9, 4, 5, 1, static_cast<std::uint8_t>(feedback ? 2 : 1), 1, 2, 0x20, 0});
    append(config, {16, 0x24, 1, 1, 0, 1, 1, 0, 0, 0, 2, 3, 0, 0, 0, 0});
    append(config, {6, 0x24, 2, 1, 4, 24});
    append(config, {7, 5, 0x01, 5, 56, 0, 1});
    if (feedback)
        append(config, {7, 5, 0x81, 0x11, 4, 0, 4});
    if (capture) {
        append(config, {9, 4, 6, 1, 1, 1, 2, 0x20, 0});
        append(config, {16, 0x24, 1, 2, 0, 1, 1, 0, 0, 0, 2, 3, 0, 0, 0, 0});
        append(config, {6, 0x24, 2, 1, 3, 24});
        append(config, {7, 5, 0x82, 5, 42, 0, 1});
    }
    return config;
}

void testDescriptors() {
    const Config first = uac1();
    const Config second = uac2(true);
    Config raw{18, 1, 0, 2, 0, 0, 0, 64, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2};
    raw.insert(raw.end(), first.begin(), first.end());
    raw.insert(raw.end(), second.begin(), second.end());
    check(findConfiguration(raw, 0).empty(), "Missing configuration should not be selected");

    const auto fs = parseConfiguration(findConfiguration(raw, 1), Speed::full);
    check(fs && fs->streams.size() == 2, "UAC1 duplex descriptors");
    const auto& playback = fs->streams[0];
    const auto& capture = fs->streams[1];
    check(playback.channels == 2 && playback.slotBytes == 2 && !playback.data.isInput(), "UAC1 playback format");
    check(capture.channels == 1 && capture.data.isInput() && capture.data.sampleRateControl, "UAC1 input format");
    check(playback.fits(48000, Speed::full) && !playback.fits(44100, Speed::full), "UAC1 fixed rate");
    check(capture.fits(44100, Speed::full) && capture.fits(48000, Speed::full) && !capture.fits(32000, Speed::full),
          "UAC1 range and endpoint rate control");
    check(fs->controlFor(playback) == fs->controlFor(capture) && fs->controlFor(capture) != nullptr,
          "UAC1 AudioControl ownership");

    const auto hs = parseConfiguration(findConfiguration(raw, 2), Speed::high);
    check(hs && hs->streams.size() == 2, "UAC2 duplex descriptors");
    check(hs->streams[0].feedback && hs->streams[0].feedback->address == 0x81,
          "UAC2 framed OUT with explicit asynchronous feedback");
    check(hs->streams[0].fits(48000, Speed::high) && !hs->streams[0].fits(96000, Speed::high),
          "UAC2 selection limits follow packet capacity");
    const auto* control = hs->controlFor(hs->streams[1]);
    check(control && control->interface == 4 && control->clocks.size() == 2 && control->clocks[0].writableRate,
          "UAC2 IAD and clock controls");
    check(control->clocks[1].selectorSources == std::vector<unsigned>{0x21}, "UAC2 clock selector source");
    check(control->terminalClocks == std::vector<std::pair<unsigned, unsigned>>{{1, 0x22}, {2, 0x21}},
          "UAC2 terminal-to-clock sources");

    const auto unsupported = parseConfiguration(uac2(false), Speed::high);
    check(unsupported && unsupported->streams.size() == 1 && unsupported->streams[0].data.isInput(),
          "Asynchronous playback without supported feedback must be excluded");
    for (std::size_t size = 0; size < second.size(); ++size)
        check(!parseConfiguration(Bytes(second).first(size), Speed::high), "Truncated config must be rejected");
    auto invalid = second;
    invalid[9] = 0;
    check(!parseConfiguration(invalid, Speed::high), "A zero-length descriptor must be rejected");
}

void testClockAndPcm() {
    PacketClock fractional(44100, Speed::full, 1);
    unsigned sum = 0;
    for (unsigned i = 0; i < 1000; ++i)
        sum += fractional.nextFrames(46);
    check(sum == 44100, "Fractional frames should accumulate rather than truncate each interval");

    PacketClock high(48000, Speed::high, 1);
    const std::array<std::uint8_t, 4> feedbackH{0, 0x04, 6, 0}; // 6.015625 frames per microframe
    const std::array<std::uint8_t, 4> badFeedback{0, 0, 7, 0};
    check(!high.applyFeedback(badFeedback), "Reject clock reports far from the selected sample rate");
    sum = 0;
    for (unsigned i = 0; i < 8000; ++i) {
        if (i % 8 == 0)
            check(high.applyFeedback(feedbackH), "Accept 16.16 high-speed feedback");
        sum += high.nextFrames(7);
    }
    check(sum > 48120 && sum <= 48125, "Asynchronous feedback must adjust outbound packet size");
    PacketClock full(48000, Speed::full, 1);
    const std::array<std::uint8_t, 3> feedbackF{0x00, 0x00, 0x0c}; // 48.0 frames per frame
    check(full.applyFeedback(feedbackF) && full.nextFrames(49) == 48, "Accept 10.14 full-speed feedback");

    std::array<std::uint8_t, 4> pcm{};
    for (const auto& [slot, bits] : {std::pair{2u, 16u}, {3u, 24u}, {4u, 24u}, {4u, 32u}}) {
        for (const float sample : {-1.0f, -0.5f, 0.0f, 0.25f, 0.75f}) {
            packPcm(sample, pcm.data(), slot, bits);
            check(unpackPcm(pcm.data(), slot) == sample, "PCM sign and left-aligned quantization");
        }
        packPcm(2.0f, pcm.data(), slot, bits);
        check(unpackPcm(pcm.data(), slot) > 0.9999f, "Positive overload clips safely");
        packPcm(std::numeric_limits<float>::quiet_NaN(), pcm.data(), slot, bits);
        check(unpackPcm(pcm.data(), slot) == 0.0f, "Nonfinite output becomes silence");
    }
}

void testOpenFailure() {
    Device device;
    check(!device.open(-1, {}) && !device.lastError().empty(), "Invalid fd must fail with an error");
    const auto nullFd = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
    check(nullFd >= 0 && !device.open(nullFd, {}), "A non-USB fd must fail without claiming any interface");
    check(::fcntl(nullFd, F_GETFD) >= 0, "A failed open must leave the caller's fd open");
    ::close(nullFd);
    device.stop();
    device.close();
    check(!device.isRunning() && device.format().outputChannels == 0, "Cleanup is safe after failed opens");
}

#if defined(ANDROID_USB_AUDIO_TEST_USBFS)

// Emulate the usbfs completion boundary so tests reach the real worker and
// teardown code. Only poll schedules a new batch of completed transfers.
struct MockUsb {
    std::deque<usbdevfs_urb*> pending;
    std::deque<usbdevfs_urb*> completed;
    std::array<unsigned, 8> alt{};
    int failSubmitAfter = -1;
    unsigned claims = 0;
    unsigned submissions = 0;
    unsigned discarded = 0;
    unsigned rate = 44100;
    unsigned capturedPackets = 0;
    unsigned sevenFramePackets = 0;
    bool playedCallbackOutput = false;

    int control(usbdevfs_ctrltransfer& req) {
        auto* data = static_cast<std::uint8_t*>(req.data);
        if (req.bRequestType == 0x80 && req.bRequest == USB_REQ_GET_CONFIGURATION && req.wLength == 1) {
            data[0] = 2;
            return 1;
        }
        if (req.wValue != 0x100 || req.bRequest != 1 || (req.wIndex & 0xffu) != 4)
            return error(EPIPE);
        const auto entity = req.wIndex >> 8u;
        if (entity == 0x22 && req.wLength == 1 && req.bRequestType == 0xa1) {
            data[0] = 1;
            return 1;
        }
        if (entity != 0x21 || req.wLength != 4)
            return error(EPIPE);
        if (req.bRequestType == 0x21)
            rate = littleEndian(Bytes(data, 4));
        else if (req.bRequestType == 0xa1) {
            for (unsigned i = 0; i < 4; ++i)
                data[i] = static_cast<std::uint8_t>(rate >> (8u * i));
        } else {
            return error(EPIPE);
        }
        return 4;
    }

    static int error(int code) {
        errno = code;
        return -1;
    }

    void complete(usbdevfs_urb& urb) {
        urb.status = 0;
        auto* bytes = static_cast<std::uint8_t*>(urb.buffer);
        for (int i = 0; i < urb.number_of_packets; ++i) {
            auto& packet = urb.iso_frame_desc[i];
            packet.status = 0;
            switch (urb.endpoint) {
            case 0x01: // PCM OUT, 2 channels in 4-byte slots
                packet.actual_length = packet.length;
                sevenFramePackets += packet.length == 56;
                for (unsigned j = 0; j < packet.length; j += 4)
                    playedCallbackOutput |= bytes[j + 3] == 0x10;
                break;
            case 0x81: // 6.015625 sample frames per microframe, as 16.16
                std::copy_n(std::array<std::uint8_t, 4>{0, 0x04, 6, 0}.begin(), 4, bytes);
                packet.actual_length = 4;
                break;
            case 0x82: // PCM IN, 2 channels in 3-byte slots
                packet.actual_length = (++capturedPackets % 64 == 0 ? 7u : 6u) * 6u;
                for (unsigned j = 0; j < packet.actual_length; j += 3) {
                    bytes[j] = bytes[j + 1] = 0;
                    bytes[j + 2] = 0x20;
                }
                break;
            }
            bytes += packet.length;
        }
    }
};

MockUsb* mock = nullptr;

struct CallbackProbe {
    bool expectsInput = true;
    std::atomic<unsigned> calls{0};
    std::atomic<bool> heardInput{false};
    std::atomic<bool> badBlock{false};

    static void process(void* self, const float* input, float* output, unsigned frames) noexcept {
        auto& probe = *static_cast<CallbackProbe*>(self);
        if ((input != nullptr) != probe.expectsInput || frames != 64)
            probe.badBlock = true;
        else if (input && input[0] == 0.25f && input[1] == 0.25f)
            probe.heardInput = true;
        std::fill_n(output, frames * 2, 0.125f);
        ++probe.calls;
    }
};

void waitForCallbacks(const CallbackProbe& probe, unsigned calls) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (probe.calls < calls && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void testUsbfsWorker() {
    MockUsb usb;
    mock = &usb;
    const auto fd = ::open("/dev/null", O_RDWR | O_CLOEXEC);
    check(fd >= 0, "Transport test fd");

    Device device;
    const auto config = uac2(true);
    if (!device.open(fd, config))
        check(false, device.lastError());
    const auto format = device.format();
    check(format.sampleRate == 48000 && format.inputChannels == 2 && format.outputChannels == 2,
          "The native worker must open both directions at a shared clock rate");
    check(usb.rate == 48000 && usb.claims == 3 && usb.alt[5] == 1 && usb.alt[6] == 1,
          "Select streaming alternates and program the current UAC2 clock");

    CallbackProbe probe;
    usb.failSubmitAfter = 2;
    check(!device.start(64, CallbackProbe::process, &probe), "USB partial submission must fail startup");
    check(usb.submissions == 2 && usb.discarded == 2 && usb.pending.empty() && usb.completed.empty(),
          "Cancel and reap every queued partial-start URB");
    check(!device.isRunning() && probe.calls == 0, "A failed start invokes no DSP callbacks");

    usb.failSubmitAfter = -1;
    if (!device.start(64, CallbackProbe::process, &probe))
        check(false, device.lastError());
    waitForCallbacks(probe, 100);
    device.close();
    const auto stats = device.statistics();
    check(probe.calls >= 100 && probe.heardInput && !probe.badBlock && usb.playedCallbackOutput,
          "Input USB PCM must reach DSP and the rendered output must reach USB");
    check(usb.sevenFramePackets > 0 && stats.completedFrames > 4800 && stats.packetErrors == 0,
          "The asynchronous OUT transfers must follow high-speed clock feedback");
    check(usb.pending.empty() && usb.completed.empty() && usb.claims == 0 && usb.alt[5] == 0 && usb.alt[6] == 0,
          "Close drains outstanding I/O, restores alternate zero, and releases all interfaces");

    const auto outputOnlyConfig = uac2(true, false);
    check(device.open(fd, outputOnlyConfig) && device.format().inputChannels == 0,
          "A playback-only device opens without a capture path");
    CallbackProbe playbackOnly{false};
    check(device.start(64, CallbackProbe::process, &playbackOnly), "Start playback without capture");
    waitForCallbacks(playbackOnly, 10);
    device.close();
    check(playbackOnly.calls >= 10 && !playbackOnly.badBlock, "Playback-only callback receives null input");
    check(::fcntl(fd, F_GETFD) >= 0, "Close must not close the caller's descriptor");
    ::close(fd);
    mock = nullptr;
}

#endif

} // namespace

#if defined(ANDROID_USB_AUDIO_TEST_USBFS)

extern "C" int __wrap_ioctl(int, unsigned long request, ...) {
    if (!mock)
        return MockUsb::error(ENOTTY);
    if (request == USBDEVFS_GET_SPEED)
        return USB_SPEED_HIGH;

    va_list args;
    va_start(args, request);
    const auto result = [&]() -> int {
        switch (request) {
        case USBDEVFS_CONTROL: return mock->control(*va_arg(args, usbdevfs_ctrltransfer*));
        case USBDEVFS_CLAIMINTERFACE: ++mock->claims; return 0;
        case USBDEVFS_RELEASEINTERFACE: --mock->claims; return 0;
        case USBDEVFS_SETINTERFACE: {
            const auto& setting = *va_arg(args, usbdevfs_setinterface*);
            mock->alt.at(setting.interface) = setting.altsetting;
            return 0;
        }
        case USBDEVFS_SUBMITURB:
            if (mock->failSubmitAfter == static_cast<int>(mock->submissions))
                return MockUsb::error(EIO);
            mock->pending.push_back(va_arg(args, usbdevfs_urb*));
            ++mock->submissions;
            return 0;
        case USBDEVFS_DISCARDURB: {
            auto* transfer = va_arg(args, usbdevfs_urb*);
            const auto queued = std::find(mock->pending.begin(), mock->pending.end(), transfer);
            if (queued == mock->pending.end())
                return MockUsb::error(EINVAL);
            mock->pending.erase(queued);
            mock->completed.push_back(transfer);
            ++mock->discarded;
            return 0;
        }
        case USBDEVFS_REAPURB:
        case USBDEVFS_REAPURBNDELAY:
            if (mock->completed.empty())
                return MockUsb::error(EAGAIN);
            *va_arg(args, usbdevfs_urb**) = mock->completed.front();
            mock->completed.pop_front();
            return 0;
        default: return MockUsb::error(ENOTTY);
        }
    }();
    va_end(args);
    return result;
}

extern "C" int __wrap_poll(pollfd* descriptors, nfds_t count, int) {
    if (!mock || count != 2)
        return MockUsb::error(EINVAL);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    for (auto* transfer : mock->pending) {
        mock->complete(*transfer);
        mock->completed.push_back(transfer);
    }
    mock->pending.clear();
    descriptors[0].revents = POLLOUT;
    return 1;
}

#endif

int main() {
    testDescriptors();
    testClockAndPcm();
    testOpenFailure();
#if defined(ANDROID_USB_AUDIO_TEST_USBFS)
    testUsbfsWorker();
#endif
}
