#include "UsbAudioDevice.h"
#include "UsbAudioDescriptors.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>
#include <limits>
#include <poll.h>
#include <pthread.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <system_error>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <utility>
#include <vector>

namespace wdf::android::usb {
namespace {

using detail::Bytes;
using detail::Control;
using detail::Endpoint;
using detail::Protocol;
using detail::Speed;
using detail::Stream;

constexpr unsigned streamQueueDepth = 3;
constexpr unsigned feedbackQueueDepth = 2;
constexpr unsigned maxPacketsPerTransfer = 8;

#if !defined(USBDEVFS_GET_SPEED)
constexpr unsigned getSpeed = _IO('U', 31);
#else
constexpr unsigned getSpeed = USBDEVFS_GET_SPEED;
#endif

// usbfs completion objects and packet memory are reused throughout streaming.
struct Transfer {
    enum class Kind { input, output, feedback };

    static void destroyUrb(usbdevfs_urb* pointer) { ::operator delete(pointer); }

    Transfer(Kind kind, const Endpoint& endpoint, Speed speed)
        : kind(kind), capacity(endpoint.packetBytes),
          packets(kind == Kind::feedback ? 1u :
              std::min(maxPacketsPerTransfer,
                  std::max(1u, detail::ticksPerSecond(speed) / (1000u * endpoint.intervalTicks)))),
          bytes(capacity * packets),
          urb(static_cast<usbdevfs_urb*>(::operator new(sizeof(usbdevfs_urb) +
              packets * sizeof(usbdevfs_iso_packet_desc))), destroyUrb) {
        std::memset(urb.get(), 0, sizeof(usbdevfs_urb) + packets * sizeof(usbdevfs_iso_packet_desc));
        urb->type = USBDEVFS_URB_TYPE_ISO;
        urb->endpoint = static_cast<unsigned char>(endpoint.address);
        urb->flags = USBDEVFS_URB_ISO_ASAP;
        urb->number_of_packets = static_cast<int>(packets);
        urb->buffer = bytes.data();
    }

    Kind kind;
    unsigned capacity;
    unsigned packets;
    std::vector<std::uint8_t> bytes;
    std::unique_ptr<usbdevfs_urb, decltype(&destroyUrb)> urb;
    bool pending = false;
};

class InputFifo {
public:
    InputFifo(unsigned channels, unsigned frames)
        : samples_(channels * frames), channels_(channels), capacity_(frames) {}

    bool push(const std::uint8_t* frame, unsigned slotBytes) noexcept {
        const bool overrun = size_ == capacity_;
        if (overrun) {
            read_ = (read_ + 1u) % capacity_;
            --size_;
        }
        auto* dst = samples_.data() + ((read_ + size_) % capacity_) * channels_;
        for (unsigned channel = 0; channel < channels_; ++channel)
            dst[channel] = detail::unpackPcm(frame + channel * slotBytes, slotBytes);
        ++size_;
        return overrun;
    }

    unsigned size() const noexcept { return size_; }

    void discard(unsigned frames) noexcept {
        read_ = (read_ + frames) % capacity_;
        size_ -= frames;
    }

    void pop(float* dst, unsigned frames) noexcept {
        const auto first = std::min(frames, capacity_ - read_);
        std::copy_n(samples_.data() + read_ * channels_, first * channels_, dst);
        std::copy_n(samples_.data(), (frames - first) * channels_, dst + first * channels_);
        discard(frames);
    }

private:
    std::vector<float> samples_;
    unsigned channels_;
    unsigned capacity_;
    unsigned read_ = 0;
    unsigned size_ = 0;
};

struct Streaming {
    struct Block {
        std::vector<float> input;
        std::vector<float> output;
    };

    static constexpr auto stopDsp = std::numeric_limits<std::uint64_t>::max();

    Streaming(const Stream& output, const std::optional<Stream>& input,
              Speed speed, unsigned rate, unsigned block, AudioCallback callback, void* context)
        : callback(callback), context(context), blockFrames(block), outputCursor(block),
          fadeFrames(std::max(1u, rate / 1500u)),
          playbackLead(std::max(block, streamQueueDepth * std::max(rate / 1000u,
              rate * output.data.intervalTicks / detail::ticksPerSecond(speed)))), leadRemaining(playbackLead),
          fifo(input ? input->channels : 0u,
               std::max({block * 4u, rate / 100u, (block + playbackLead) * 2u})),
          blocks((playbackLead + block - 1u) / block + 2u),
          lastOutput(output.channels),
          outputClock(rate, speed, output.data.intervalTicks) {
        for (auto& item : blocks) {
            if (input)
                item.input.resize(input->channels * block);
            item.output.resize(output.channels * block);
        }
        const auto enqueue = [&](Transfer::Kind kind, const Endpoint& endpoint, unsigned count) {
            for (unsigned i = 0; i < count; ++i)
                transfers.push_back(std::make_unique<Transfer>(kind, endpoint, speed));
        };
        if (input)
            enqueue(Transfer::Kind::input, input->data, streamQueueDepth);
        if (output.feedback)
            enqueue(Transfer::Kind::feedback, *output.feedback, feedbackQueueDepth);
        enqueue(Transfer::Kind::output, output.data, streamQueueDepth);
    }

    Block& block(std::uint64_t index) noexcept { return blocks[index % blocks.size()]; }

    void queueDsp(bool hasInput) noexcept {
        if (captureOverflow)
            return;
        const auto previous = queued;
        while (queued - played < blocks.size() && (hasInput ? fifo.size() >= blockFrames : queued - played < 2u)) {
            if (hasInput)
                fifo.pop(block(queued).input.data(), blockFrames);
            ++queued;
        }
        if (queued != previous) {
            requested.store(queued, std::memory_order_release);
            requested.notify_one();
        }
    }

    void runDsp(std::atomic<int>& nice) noexcept {
        pthread_setname_np(pthread_self(), "USB audio DSP");
        ::setpriority(PRIO_PROCESS, 0, -10);
        nice.store(::getpriority(PRIO_PROCESS, 0), std::memory_order_relaxed);
        for (std::uint64_t next = 0;; ++next) {
            requested.wait(next, std::memory_order_acquire);
            if (requested.load(std::memory_order_acquire) == stopDsp)
                return;
            auto& item = block(next);
            std::fill(item.output.begin(), item.output.end(), 0.0f);
            callback(context, item.input.empty() ? nullptr : item.input.data(), item.output.data(), blockFrames);
            ready.store(next + 1u, std::memory_order_release);
        }
    }

    void stopProcessing() noexcept {
        requested.store(stopDsp, std::memory_order_release);
        requested.notify_one();
    }

    AudioCallback callback;
    void* context;
    const unsigned blockFrames;
    unsigned outputCursor;
    const unsigned fadeFrames;
    const unsigned playbackLead;
    unsigned leadRemaining;
    bool renderStarted = false;
    bool captureOverflow = false;
    unsigned fadeOutRemaining = 0;
    unsigned fadeInRemaining = 0;
    std::uint64_t lastCaptureAt = 0;
    InputFifo fifo;
    // USB owns queue and playback cursors. The DSP thread advances ready after writing a block.
    std::vector<Block> blocks;
    std::uint64_t queued = 0;
    std::uint64_t played = 0;
    std::uint64_t discardBefore = 0;
    std::atomic<std::uint64_t> requested{0};
    std::atomic<std::uint64_t> ready{0};
    std::vector<float> lastOutput;
    detail::PacketClock outputClock;
    std::vector<std::unique_ptr<Transfer>> transfers;
};

enum class WorkerError { none, disconnected, stalled, captureStalled, poll, reap, submit };

} // namespace

struct Device::Impl {
    struct InterfaceClaim {
        unsigned number;
        bool detached;
        bool hasAlternate = false;
    };

    int fd = -1;
    int wake = -1;
    Speed speed = Speed::full;
    Format format;
    Stream output;
    std::optional<Stream> input;
    std::array<InterfaceClaim, 3> claims{};
    unsigned claimCount = 0;
    std::string localError;
    std::atomic<WorkerError> workerError{WorkerError::none};
    std::atomic<int> workerErrno{0};
    std::atomic<bool> running{false};
    std::atomic<bool> stopRequested{false};
    std::atomic<std::uint64_t> completedFrames{0};
    std::atomic<std::uint64_t> inputOverruns{0};
    std::atomic<std::uint64_t> inputUnderruns{0};
    std::atomic<std::uint64_t> packetErrors{0};
    std::atomic<std::uint64_t> captureRecoveries{0};
    std::atomic<std::uint64_t> playbackRecoveries{0};
    std::atomic<int> usbNice{0};
    std::atomic<int> dspNice{0};
    std::thread worker;
    std::thread dspWorker;
    std::unique_ptr<Streaming> streaming;

    bool fail(const char* message, int error = 0) {
        localError = message;
        if (error != 0)
            localError += ": " + std::system_category().message(error);
        return false;
    }

    int control(unsigned type, unsigned request, unsigned value, unsigned index, std::span<std::uint8_t> data) const noexcept {
        usbdevfs_ctrltransfer transfer{static_cast<std::uint8_t>(type), static_cast<std::uint8_t>(request),
            static_cast<std::uint16_t>(value), static_cast<std::uint16_t>(index),
            static_cast<std::uint16_t>(data.size()), 500u, data.data()};
        return ::ioctl(fd, USBDEVFS_CONTROL, &transfer);
    }

    std::vector<std::uint8_t> readConfiguration(unsigned active) const {
        std::array<std::uint8_t, 18> device{};
        if (control(0x80, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8u, 0, device) !=
            static_cast<int>(device.size()))
            return {};
        for (unsigned i = 0; i < device[17]; ++i) {
            std::array<std::uint8_t, 9> header{};
            const auto descriptor = (USB_DT_CONFIG << 8u) | i;
            if (control(0x80, USB_REQ_GET_DESCRIPTOR, descriptor, 0, header) != static_cast<int>(header.size()) ||
                header[5] != active)
                continue;
            const auto size = detail::littleEndian(Bytes(header).subspan(2, 2));
            if (size < header.size())
                return {};
            std::vector<std::uint8_t> config(size);
            if (control(0x80, USB_REQ_GET_DESCRIPTOR, descriptor, 0, config) == static_cast<int>(size))
                return config;
            return {};
        }
        return {};
    }

    bool claim(unsigned number) {
        for (unsigned i = 0; i < claimCount; ++i) {
            if (claims[i].number == number)
                return true;
        }
        if (::ioctl(fd, USBDEVFS_CLAIMINTERFACE, &number) == 0) {
            claims[claimCount++] = {number, false};
            return true;
        }
        if (errno == EBUSY) {
            // Never detach an unrelated kernel driver from a composite device.
            usbdevfs_disconnect_claim request{number, USBDEVFS_DISCONNECT_CLAIM_IF_DRIVER, "snd-usb-audio"};
            if (::ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &request) == 0) {
                claims[claimCount++] = {number, true};
                return true;
            }
        }
        return fail("Could not claim the USB audio interface", errno);
    }

    bool setAlternate(const Stream& stream) {
        usbdevfs_setinterface setting{stream.interface, stream.alternate};
        if (::ioctl(fd, USBDEVFS_SETINTERFACE, &setting) < 0)
            return fail("Could not select the USB audio stream format", errno);
        for (unsigned i = 0; i < claimCount; ++i) {
            if (claims[i].number == stream.interface)
                claims[i].hasAlternate = true;
        }
        return true;
    }

    void releaseInterfaces() noexcept {
        for (unsigned i = claimCount; i > 0; --i) {
            auto& claim = claims[i - 1u];
            if (claim.hasAlternate) {
                usbdevfs_setinterface setting{claim.number, 0u};
                ::ioctl(fd, USBDEVFS_SETINTERFACE, &setting);
            }
            ::ioctl(fd, USBDEVFS_RELEASEINTERFACE, &claim.number);
        }
        for (unsigned i = 0; i < claimCount; ++i) {
            if (claims[i].detached) {
                usbdevfs_ioctl reconnect{static_cast<int>(claims[i].number), USBDEVFS_CONNECT, nullptr};
                ::ioctl(fd, USBDEVFS_IOCTL, &reconnect);
            }
        }
        claimCount = 0;
    }

    const detail::Clock* clockFor(const Control& owner, const Stream& stream) const {
        const auto terminal = std::find_if(owner.terminalClocks.begin(), owner.terminalClocks.end(),
            [&](const auto& entry) { return entry.first == stream.terminal; });
        if (terminal == owner.terminalClocks.end())
            return nullptr;
        unsigned id = terminal->second;
        for (std::size_t depth = 0; depth < owner.clocks.size(); ++depth) {
            const auto clock = std::find_if(owner.clocks.begin(), owner.clocks.end(),
                [id](const detail::Clock& entry) { return entry.id == id; });
            if (clock == owner.clocks.end())
                return nullptr;
            if (clock->selectorSources.empty())
                return &*clock;
            std::array<std::uint8_t, 1> selected{};
            const auto index = (clock->id << 8u) | owner.interface;
            if (control(0xa1, 1, 0x100, index, selected) != 1 || selected[0] == 0 ||
                selected[0] > clock->selectorSources.size())
                return nullptr;
            id = clock->selectorSources[selected[0] - 1u];
        }
        return nullptr; // Selector cycle or unsupported clock topology.
    }

    bool clockRate(const Control& owner, const detail::Clock& clock, unsigned rate) {
        const auto index = (clock.id << 8u) | owner.interface;
        std::array<std::uint8_t, 4> bytes{};
        const auto atRate = [&] {
            return control(0xa1, 1, 0x100, index, bytes) == 4 && detail::littleEndian(bytes) == rate;
        };
        if (atRate())
            return true;
        if (!clock.writableRate)
            return fail("The USB audio clock cannot be set to the requested sample rate");
        for (unsigned i = 0; i < bytes.size(); ++i)
            bytes[i] = static_cast<std::uint8_t>(rate >> (8u * i));
        if (control(0x21, 1, 0x100, index, bytes) != 4 || !atRate())
            return fail("The USB audio clock did not accept the requested sample rate");
        return true;
    }

    bool endpointRate(const Stream& stream, unsigned rate) {
        if (!stream.data.sampleRateControl)
            return true;
        std::array<std::uint8_t, 3> bytes{};
        for (unsigned i = 0; i < bytes.size(); ++i)
            bytes[i] = static_cast<std::uint8_t>(rate >> (8u * i));
        if (control(0x22, 1, 0x100, stream.data.address, bytes) != 3)
            return fail("The USB streaming endpoint did not accept the requested sample rate");
        return true;
    }

    bool configure(const Control& owner, const Stream& playback, const Stream* capture, unsigned rate) {
        if (!claim(owner.interface) || !claim(playback.interface) || (capture && !claim(capture->interface)))
            return false;
        if (owner.protocol == Protocol::uac2) {
            const auto* outputClock = clockFor(owner, playback);
            if (!outputClock || (capture && clockFor(owner, *capture) != outputClock))
                return fail("The USB streams do not expose a supported shared UAC2 clock");
            if (!clockRate(owner, *outputClock, rate))
                return false;
        }
        if (!setAlternate(playback) || (capture && !setAlternate(*capture)))
            return false;
        if (owner.protocol == Protocol::uac1 &&
            (!endpointRate(playback, rate) || (capture && !endpointRate(*capture, rate))))
            return false;
        output = playback;
        input = capture ? std::optional<Stream>(*capture) : std::nullopt;
        format = {rate, input ? input->channels : 0u, output.channels};
        localError.clear();
        return true;
    }

    bool select(const detail::Topology& topology, unsigned rate) {
        std::vector<const Stream*> inputs;
        std::vector<const Stream*> outputs;
        for (const auto& stream : topology.streams) {
            if (stream.fits(rate, speed) && topology.controlFor(stream))
                (stream.data.isInput() ? inputs : outputs).push_back(&stream);
        }
        const auto prefer = [](const Stream* a, const Stream* b) {
            const auto score = [](const Stream& stream) {
                return std::tuple{stream.data.intervalTicks, stream.channels == 2 ? 0u : stream.channels,
                                  32u - stream.sampleBits};
            };
            return score(*a) < score(*b);
        };
        std::stable_sort(outputs.begin(), outputs.end(), prefer);
        std::stable_sort(inputs.begin(), inputs.end(), prefer);
        for (const auto* playback : outputs) {
            const auto* owner = topology.controlFor(*playback);
            for (const auto* capture : inputs) {
                if (capture->interface == playback->interface || topology.controlFor(*capture) != owner)
                    continue;
                if (configure(*owner, *playback, capture, rate))
                    return true;
                releaseInterfaces();
            }
        }
        for (const auto* playback : outputs) {
            if (configure(*topology.controlFor(*playback), *playback, nullptr, rate))
                return true;
            releaseInterfaces();
        }
        return localError.empty() ? fail("No supported PCM playback stream at the requested sample rate") : false;
    }

    bool open(int sourceFd, Bytes raw, unsigned rate) {
        fd = ::fcntl(sourceFd, F_DUPFD_CLOEXEC, 0);
        if (fd < 0)
            return fail("Could not duplicate the Android USB device descriptor", errno);
        const auto busSpeed = ::ioctl(fd, getSpeed);
        if (busSpeed < 0)
            return fail("Could not determine the USB connection speed", errno);
        if (busSpeed != USB_SPEED_FULL && busSpeed != USB_SPEED_HIGH)
            return fail("Direct USB audio requires a full-speed or high-speed USB connection");
        speed = busSpeed == USB_SPEED_HIGH ? Speed::high : Speed::full;
        std::array<std::uint8_t, 1> configValue{};
        if (control(0x80, USB_REQ_GET_CONFIGURATION, 0, 0, configValue) != 1 || configValue[0] == 0)
            return fail("Could not read the active USB device configuration");
        auto descriptors = detail::findConfiguration(raw, configValue[0]);
        std::vector<std::uint8_t> fetched;
        if (descriptors.empty()) {
            fetched = readConfiguration(configValue[0]);
            descriptors = detail::findConfiguration(fetched, configValue[0]);
        }
        const auto topology = detail::parseConfiguration(descriptors, speed);
        if (!topology)
            return fail("The active USB configuration descriptors could not be read");
        return select(*topology, rate);
    }

    void render(std::uint8_t* target, unsigned frames, bool prime) noexcept {
        auto& state = *streaming;
        const auto stride = output.frameBytes();
        if (prime) {
            std::fill_n(target, frames * stride, std::uint8_t{0});
            return;
        }

        while (frames > 0) {
            if (state.outputCursor == state.blockFrames) {
                if (state.captureOverflow) {
                    if (state.renderStarted) {
                        captureRecoveries.fetch_add(1, std::memory_order_relaxed);
                        state.renderStarted = false;
                        state.fadeOutRemaining = state.fadeFrames;
                    }
                    state.discardBefore = state.queued;
                    state.fifo.discard(state.fifo.size() - state.blockFrames);
                    state.captureOverflow = false;
                    state.leadRemaining = state.playbackLead;
                }
                if (state.played < state.discardBefore)
                    state.played = std::min(state.ready.load(std::memory_order_acquire), state.discardBefore);
                state.queueDsp(input.has_value());
                const auto readyBlocks = state.ready.load(std::memory_order_acquire);

                if (state.renderStarted && readyBlocks == state.played) {
                    if (input && state.queued == state.played) {
                        inputUnderruns.fetch_add(state.blockFrames - state.fifo.size(), std::memory_order_relaxed);
                        captureRecoveries.fetch_add(1, std::memory_order_relaxed);
                        state.leadRemaining = state.playbackLead;
                    } else {
                        playbackRecoveries.fetch_add(1, std::memory_order_relaxed);
                    }
                    state.renderStarted = false;
                    state.fadeOutRemaining = state.fadeFrames;
                }
                if (state.fadeOutRemaining > 0) {
                    const auto count = std::min(frames, state.fadeOutRemaining);
                    for (unsigned frame = 0; frame < count; ++frame) {
                        const auto gain = static_cast<float>(--state.fadeOutRemaining) / state.fadeFrames;
                        for (unsigned channel = 0; channel < output.channels; ++channel)
                            detail::packPcm(state.lastOutput[channel] * gain,
                                target + frame * stride + channel * output.slotBytes,
                                output.slotBytes, output.sampleBits);
                    }
                    target += count * stride;
                    frames -= count;
                    continue;
                }
                if (state.leadRemaining > 0 && state.queued > std::max(state.played, state.discardBefore)) {
                    const auto count = std::min(frames, state.leadRemaining);
                    std::fill_n(target, count * stride, std::uint8_t{0});
                    state.leadRemaining -= count;
                    target += count * stride;
                    frames -= count;
                    continue;
                }
                if (state.played < state.discardBefore || readyBlocks == state.played) {
                    std::fill_n(target, frames * stride, std::uint8_t{0});
                    return;
                }
                if (!state.renderStarted) {
                    state.renderStarted = true;
                    state.fadeInRemaining = state.fadeFrames;
                }
                state.outputCursor = 0;
            }
            const auto count = std::min(frames, state.blockFrames - state.outputCursor);
            const auto* samples = state.block(state.played).output.data() + state.outputCursor * output.channels;
            for (unsigned frame = 0; frame < count; ++frame) {
                const float gain = state.fadeInRemaining > 0
                    ? 1.0f - static_cast<float>(--state.fadeInRemaining) / state.fadeFrames : 1.0f;
                for (unsigned channel = 0; channel < output.channels; ++channel) {
                    const auto index = frame * output.channels + channel;
                    const auto sample = samples[index] * gain;
                    state.lastOutput[channel] = sample;
                    detail::packPcm(sample, target + index * output.slotBytes,
                                    output.slotBytes, output.sampleBits);
                }
            }
            state.outputCursor += count;
            if (state.outputCursor == state.blockFrames)
                ++state.played;
            frames -= count;
            target += count * stride;
        }
    }

    bool submit(Transfer& transfer, bool prime = false) noexcept {
        auto* urb = transfer.urb.get();
        unsigned offset = 0;
        for (unsigned i = 0; i < transfer.packets; ++i) {
            auto& packet = urb->iso_frame_desc[i];
            unsigned bytes = transfer.capacity;
            if (transfer.kind == Transfer::Kind::output) {
                const auto frames = streaming->outputClock.nextFrames(transfer.capacity / output.frameBytes());
                bytes = frames * output.frameBytes();
                render(transfer.bytes.data() + offset, frames, prime);
            }
            packet = {bytes, 0u, 0u};
            offset += bytes;
        }
        urb->buffer_length = static_cast<int>(offset);
        if (::ioctl(fd, USBDEVFS_SUBMITURB, urb) < 0)
            return false;
        transfer.pending = true;
        return true;
    }

    Transfer* ownTransfer(usbdevfs_urb* urb) const noexcept {
        for (const auto& transfer : streaming->transfers) {
            if (transfer->urb.get() == urb)
                return transfer.get();
        }
        return nullptr;
    }

    void consume(Transfer& transfer) noexcept {
        const auto* urb = transfer.urb.get();
        unsigned offset = 0;
        for (unsigned i = 0; i < transfer.packets; ++i) {
            const auto& packet = urb->iso_frame_desc[i];
            const auto* data = transfer.bytes.data() + offset;
            offset += packet.length;
            if (packet.status != 0 || packet.actual_length > packet.length) {
                packetErrors.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            if (transfer.kind == Transfer::Kind::feedback) {
                if (!streaming->outputClock.applyFeedback(Bytes(data, packet.actual_length)))
                    packetErrors.fetch_add(1, std::memory_order_relaxed);
            } else if (transfer.kind == Transfer::Kind::input) {
                const auto stride = input->frameBytes();
                if (packet.actual_length % stride != 0) {
                    packetErrors.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                for (unsigned start = 0; start < packet.actual_length; start += stride) {
                    if (streaming->fifo.push(data + start, input->slotBytes)) {
                        inputOverruns.fetch_add(1, std::memory_order_relaxed);
                        streaming->captureOverflow = true;
                    }
                }
                if (packet.actual_length != 0)
                    streaming->lastCaptureAt = completedFrames.load(std::memory_order_relaxed);
            } else {
                completedFrames.fetch_add(packet.actual_length / output.frameBytes(), std::memory_order_relaxed);
            }
        }
    }

    void cancelTransfers() noexcept {
        if (!streaming)
            return;
        for (auto& transfer : streaming->transfers) {
            if (!transfer->pending)
                continue;
            while (::ioctl(fd, USBDEVFS_DISCARDURB, transfer->urb.get()) < 0 && errno == EINTR) {}
        }
        const auto hasPending = [&] {
            return std::any_of(streaming->transfers.begin(), streaming->transfers.end(),
                               [](const auto& entry) { return entry->pending; });
        };
        while (hasPending()) {
            usbdevfs_urb* urb = nullptr;
            if (::ioctl(fd, USBDEVFS_REAPURB, &urb) == 0) {
                if (auto* transfer = ownTransfer(urb))
                    transfer->pending = false;
            } else if (errno != EINTR) {
                break; // On disconnect usbfs has canceled and completed every outstanding URB.
            }
        }
    }

    WorkerError run() noexcept {
        pthread_setname_np(pthread_self(), "USB audio");
        ::setpriority(PRIO_PROCESS, 0, -16); // Best effort on Android; the kernel may deny it.
        usbNice.store(::getpriority(PRIO_PROCESS, 0), std::memory_order_relaxed);
        std::array<pollfd, 2> descriptors{{{fd, POLLOUT, 0}, {wake, POLLIN, 0}}};

        while (!stopRequested.load(std::memory_order_acquire)) {
            const auto ready = ::poll(descriptors.data(), descriptors.size(), 1000);
            if (stopRequested.load(std::memory_order_acquire))
                break;
            if (ready == 0)
                return WorkerError::stalled;
            if (ready < 0) {
                if (errno == EINTR)
                    continue;
                return WorkerError::poll;
            }
            if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL))
                return WorkerError::disconnected;

            // Consume the available capture and feedback packets before generating
            // the next output packets using their audio and clock updates.
            std::array<Transfer*, streamQueueDepth> playback{};
            unsigned playbackCount = 0;
            while (!stopRequested.load(std::memory_order_acquire)) {
                usbdevfs_urb* urb = nullptr;
                if (::ioctl(fd, USBDEVFS_REAPURBNDELAY, &urb) < 0) {
                    if (errno == EINTR)
                        continue;
                    if (errno != EAGAIN)
                        return errno == ENODEV ? WorkerError::disconnected : WorkerError::reap;
                    break;
                }
                auto* transfer = ownTransfer(urb);
                if (!transfer)
                    continue;
                transfer->pending = false;
                if (urb->status == -ENODEV || urb->status == -ESHUTDOWN)
                    return WorkerError::disconnected;
                if (urb->status == 0)
                    consume(*transfer);
                else
                    packetErrors.fetch_add(1, std::memory_order_relaxed);
                if (transfer->kind == Transfer::Kind::output)
                    playback[playbackCount++] = transfer;
                else if (!submit(*transfer))
                    return errno == ENODEV ? WorkerError::disconnected : WorkerError::submit;
            }
            streaming->queueDsp(input.has_value());
            for (unsigned i = 0; i < playbackCount && !stopRequested.load(std::memory_order_acquire); ++i) {
                if (!submit(*playback[i]))
                    return errno == ENODEV ? WorkerError::disconnected : WorkerError::submit;
            }
            if (input && completedFrames.load(std::memory_order_relaxed) - streaming->lastCaptureAt > format.sampleRate)
                return WorkerError::captureStalled;
        }
        return WorkerError::none;
    }

    bool start(unsigned blockFrames, AudioCallback callback, void* context) {
        if (fd < 0 || format.outputChannels == 0 || running.load(std::memory_order_acquire) ||
            blockFrames == 0 || blockFrames > 4096 || !callback)
            return fail("Invalid direct USB audio start request");
        stop();
        localError.clear();
        workerError.store(WorkerError::none, std::memory_order_relaxed);
        completedFrames.store(0, std::memory_order_relaxed);
        inputOverruns.store(0, std::memory_order_relaxed);
        inputUnderruns.store(0, std::memory_order_relaxed);
        packetErrors.store(0, std::memory_order_relaxed);
        captureRecoveries.store(0, std::memory_order_relaxed);
        playbackRecoveries.store(0, std::memory_order_relaxed);
        usbNice.store(0, std::memory_order_relaxed);
        dspNice.store(0, std::memory_order_relaxed);
        stopRequested.store(false, std::memory_order_release);
        wake = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
        if (wake < 0)
            return fail("Could not create the direct USB audio worker signal", errno);
        try {
            streaming = std::make_unique<Streaming>(output, input, speed, format.sampleRate, blockFrames, callback, context);
            for (auto& transfer : streaming->transfers) {
                if (!submit(*transfer, true)) {
                    const auto error = errno;
                    cancelTransfers();
                    stop();
                    return fail("Could not submit the initial USB audio transfers", error);
                }
            }
            dspWorker = std::thread([this] { streaming->runDsp(dspNice); });
            running.store(true, std::memory_order_release);
            worker = std::thread([this] {
                const auto error = run();
                workerErrno.store(errno, std::memory_order_relaxed);
                streaming->stopProcessing();
                cancelTransfers();
                workerError.store(error, std::memory_order_release);
                running.store(false, std::memory_order_release);
            });
        } catch (...) {
            cancelTransfers();
            stop();
            return fail("Could not allocate or start the direct USB audio worker");
        }
        return true;
    }

    void stop() noexcept {
        stopRequested.store(true, std::memory_order_release);
        if (wake >= 0) {
            const std::uint64_t signal = 1;
            const auto result = ::write(wake, &signal, sizeof(signal));
            static_cast<void>(result);
        }
        if (worker.joinable())
            worker.join();
        if (dspWorker.joinable()) {
            streaming->stopProcessing();
            dspWorker.join();
        }
        if (wake >= 0) {
            ::close(wake);
            wake = -1;
        }
        streaming.reset();
        running.store(false, std::memory_order_release);
    }

    void close() noexcept {
        stop();
        if (fd >= 0) {
            releaseInterfaces();
            ::close(fd);
            fd = -1;
        }
        format = {};
        output = {};
        input.reset();
    }

    std::string lastError() const {
        if (!localError.empty())
            return localError;
        const auto error = workerError.load(std::memory_order_acquire);
        switch (error) {
        case WorkerError::none: return {};
        case WorkerError::disconnected: return "The USB audio interface disconnected";
        case WorkerError::stalled: return "The USB audio interface stopped returning transfers";
        case WorkerError::captureStalled: return "The USB audio input stopped delivering frames";
        case WorkerError::poll: return "USB audio poll failed: " + std::system_category().message(workerErrno.load());
        case WorkerError::reap: return "Could not receive a completed USB audio transfer: " + std::system_category().message(workerErrno.load());
        case WorkerError::submit: return "Could not queue a USB audio transfer: " + std::system_category().message(workerErrno.load());
        }
        return {};
    }
};

Device::Device() : impl_(std::make_unique<Impl>()) {}
Device::~Device() { impl_->close(); }

bool Device::open(int fd, std::span<const std::uint8_t> rawDescriptors, unsigned sampleRate) {
    impl_->close();
    impl_->localError.clear();
    impl_->workerError.store(WorkerError::none, std::memory_order_relaxed);
    if (sampleRate < 8000 || sampleRate > 384000)
        return impl_->fail("Unsupported USB audio sample rate");
    try {
        if (impl_->open(fd, rawDescriptors, sampleRate))
            return true;
    } catch (...) {
        impl_->fail("Could not allocate the USB audio configuration");
    }
    impl_->close();
    return false;
}

bool Device::start(unsigned blockFrames, AudioCallback callback, void* context) {
    return impl_->start(blockFrames, callback, context);
}

void Device::stop() noexcept { impl_->stop(); }
void Device::close() noexcept { impl_->close(); }
Format Device::format() const noexcept { return impl_->format; }
bool Device::isRunning() const noexcept { return impl_->running.load(std::memory_order_acquire); }

Statistics Device::statistics() const noexcept {
    return {impl_->completedFrames.load(std::memory_order_relaxed), impl_->inputOverruns.load(std::memory_order_relaxed),
            impl_->inputUnderruns.load(std::memory_order_relaxed), impl_->packetErrors.load(std::memory_order_relaxed),
            impl_->captureRecoveries.load(std::memory_order_relaxed),
            impl_->playbackRecoveries.load(std::memory_order_relaxed),
            impl_->usbNice.load(std::memory_order_relaxed), impl_->dspNice.load(std::memory_order_relaxed)};
}

std::string Device::lastError() const { return impl_->lastError(); }

} // namespace wdf::android::usb
