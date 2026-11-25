#include "UsbAudioDescriptors.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace wdf::android::usb::detail {
namespace {

constexpr unsigned configDescriptor = 2;
constexpr unsigned interfaceDescriptor = 4;
constexpr unsigned endpointDescriptor = 5;
constexpr unsigned associationDescriptor = 11;
constexpr unsigned classInterfaceDescriptor = 0x24;
constexpr unsigned classEndpointDescriptor = 0x25;
// Common denominator for USB full/high speed and 10.14/16.16 feedback.
constexpr std::int64_t framesUnit = 8192000;

struct Association {
    unsigned first;
    unsigned count;
};

struct Alternate {
    Stream stream;
    std::vector<Endpoint> endpoints;
    bool pcm = false;
    bool sawFormat = false;
    bool lastEndpointWasIsochronous = false;
};

void parseControl(Control& control, Bytes desc) {
    if (control.protocol == Protocol::uac1) {
        if (desc[2] == 1 && desc.size() >= 8 && desc.size() >= 8u + desc[7]) {
            control.streams.assign(desc.begin() + 8, desc.begin() + 8 + desc[7]);
        }
        return;
    }

    switch (desc[2]) {
    case 2: // Input Terminal
        if (desc.size() >= 17)
            control.terminalClocks.emplace_back(desc[3], desc[7]);
        break;
    case 3: // Output Terminal
        if (desc.size() >= 12)
            control.terminalClocks.emplace_back(desc[3], desc[8]);
        break;
    case 10: // Clock Source
        if (desc.size() >= 8 && ((desc[5] & 3u) == 1u || (desc[5] & 3u) == 3u))
            control.clocks.push_back({desc[3], (desc[5] & 3u) == 3u, {}});
        break;
    case 11: // Clock Selector. The live selection is read when opening the device.
        if (desc.size() >= 7 && desc[4] > 0 && desc.size() >= 7u + desc[4]) {
            Clock clock{desc[3], false, {}};
            clock.selectorSources.assign(desc.begin() + 5, desc.begin() + 5 + desc[4]);
            control.clocks.push_back(std::move(clock));
        }
        break;
    }
}

void parseStreaming(Alternate& alt, Bytes desc) {
    auto& stream = alt.stream;
    if (desc[2] == 1) { // AS General
        if (stream.protocol == Protocol::uac1 && desc.size() >= 7) {
            stream.terminal = desc[3];
            alt.pcm = littleEndian(desc.subspan(5, 2)) == 1;
        } else if (stream.protocol == Protocol::uac2 && desc.size() >= 16) {
            stream.terminal = desc[3];
            stream.channels = desc[10];
            alt.pcm = desc[5] == 1 && (desc[6] & 1u) != 0;
        }
    } else if (desc[2] == 2 && desc.size() >= 6 && desc[3] == 1) { // Format Type I
        if (stream.protocol == Protocol::uac1 && desc.size() >= 8) {
            stream.channels = desc[4];
            stream.slotBytes = desc[5];
            stream.sampleBits = desc[6];
            if (desc[7] == 0 && desc.size() >= 14) {
                stream.minRate = littleEndian(desc.subspan(8, 3));
                stream.maxRate = littleEndian(desc.subspan(11, 3));
                alt.sawFormat = stream.minRate != 0 && stream.minRate <= stream.maxRate;
            } else if (desc[7] > 0 && desc.size() >= 8u + 3u * desc[7]) {
                for (unsigned i = 0; i < desc[7]; ++i)
                    stream.rates.push_back(littleEndian(desc.subspan(8 + 3 * i, 3)));
                alt.sawFormat = true;
            }
        } else if (stream.protocol == Protocol::uac2) {
            stream.slotBytes = desc[4];
            stream.sampleBits = desc[5];
            alt.sawFormat = true;
        }
    }
}

bool parseEndpoint(Alternate& alt, Bytes desc, Speed speed) {
    if (desc.size() < 7 || (desc[3] & 3u) != 1u || (desc[2] & 15u) == 0 || (desc[2] & 0x70u) != 0)
        return false;

    // Both full- and high-speed isochronous intervals are powers of two.
    if (desc[6] == 0 || desc[6] > 16)
        return false;
    const auto packetSize = littleEndian(desc.subspan(4, 2));
    const auto transactions = (packetSize >> 11u) & 3u;
    if (transactions == 3u || (transactions != 0 && speed != Speed::high))
        return false;
    const auto bytes = (packetSize & 0x7ffu) * (transactions + 1);
    if (bytes == 0 || (speed == Speed::full && bytes > 1023))
        return false;

    alt.endpoints.push_back({desc[2], bytes, 1u << (desc[6] - 1u),
                             (desc[3] >> 2u) & 3u, (desc[3] >> 4u) & 3u,
                             desc.size() >= 9 ? static_cast<unsigned>(desc[8]) : 0u, false});
    return true;
}

void finishAlternate(Alternate& alt, Topology& result) {
    auto& stream = alt.stream;
    if (stream.alternate == 0 || !alt.pcm || !alt.sawFormat || stream.channels == 0 ||
        stream.channels > 32 || stream.slotBytes < 2 || stream.slotBytes > 4 ||
        stream.sampleBits < 16 || stream.sampleBits > stream.slotBytes * 8u)
        return;

    const auto data = std::find_if(alt.endpoints.begin(), alt.endpoints.end(), [&](const Endpoint& ep) {
        if (!ep.isInput())
            return ep.usage == 0;
        if (ep.usage != 0 && ep.usage != 2)
            return false;
        return std::none_of(alt.endpoints.begin(), alt.endpoints.end(), [&](const Endpoint& dest) {
            return !dest.isInput() && dest.syncAddress == ep.address;
        });
    });
    if (data == alt.endpoints.end())
        return;
    stream.data = *data;
    if (!data->isInput() && data->synchronization == 1u) {
        const auto feedback = std::find_if(alt.endpoints.begin(), alt.endpoints.end(), [&](const Endpoint& ep) {
            return ep.isInput() && (ep.usage == 1 || ep.address == data->syncAddress) &&
                   ep.packetBytes >= 3 && ep.packetBytes <= 4;
        });
        if (feedback == alt.endpoints.end())
            return; // Asynchronous playback needs a feedback source.
        stream.feedback = *feedback;
    }
    result.streams.push_back(std::move(stream));
}

} // namespace

unsigned littleEndian(Bytes bytes) noexcept {
    unsigned value = 0;
    for (std::size_t i = 0; i < bytes.size() && i < 4; ++i)
        value |= static_cast<unsigned>(bytes[i]) << (8u * i);
    return value;
}

unsigned ticksPerSecond(Speed speed) noexcept {
    return speed == Speed::high ? 8000u : 1000u;
}

Bytes findConfiguration(Bytes raw, unsigned value) noexcept {
    while (raw.size() >= 2 && raw[0] >= 2 && raw[0] <= raw.size()) {
        if (raw[1] == configDescriptor && raw[0] >= 9) {
            const auto total = littleEndian(raw.subspan(2, 2));
            if (total < raw[0] || total > raw.size())
                return {};
            if (raw[5] == value)
                return raw.first(total);
            raw = raw.subspan(total);
        } else {
            raw = raw.subspan(raw[0]);
        }
    }
    return {};
}

std::optional<Topology> parseConfiguration(Bytes config, Speed speed) {
    if (config.size() < 9 || config[0] < 9 || config[1] != configDescriptor ||
        config[0] > config.size() || littleEndian(config.subspan(2, 2)) != config.size())
        return std::nullopt;

    Topology result;
    std::vector<Association> associations;
    std::optional<Alternate> alt;
    Control* control = nullptr;
    for (auto rest = config.subspan(config[0]); !rest.empty(); rest = rest.subspan(rest[0])) {
        if (rest.size() < 2 || rest[0] < 2 || rest[0] > rest.size())
            return std::nullopt;
        const auto desc = rest.first(rest[0]);

        if (desc[1] == associationDescriptor && desc.size() >= 8 && desc[4] == 1) {
            associations.push_back({desc[2], desc[3]});
        } else if (desc[1] == interfaceDescriptor) {
            if (alt)
                finishAlternate(*alt, result);
            alt.reset();
            control = nullptr;
            if (desc.size() < 9 || desc[5] != 1 || (desc[7] != 0 && desc[7] != 0x20))
                continue;
            const auto protocol = desc[7] == 0x20 ? Protocol::uac2 : Protocol::uac1;
            if (desc[6] == 1 && desc[3] == 0) {
                result.controls.push_back({desc[2], protocol, {}, {}, {}});
                control = &result.controls.back();
            } else if (desc[6] == 2 && desc[3] != 0) {
                alt.emplace();
                alt->stream.interface = desc[2];
                alt->stream.alternate = desc[3];
                alt->stream.protocol = protocol;
            }
        } else if (desc[1] == classInterfaceDescriptor && desc.size() >= 3) {
            if (control)
                parseControl(*control, desc);
            else if (alt)
                parseStreaming(*alt, desc);
        } else if (alt && desc[1] == endpointDescriptor) {
            alt->lastEndpointWasIsochronous = parseEndpoint(*alt, desc, speed);
        } else if (alt && alt->stream.protocol == Protocol::uac1 && alt->lastEndpointWasIsochronous &&
                   desc[1] == classEndpointDescriptor && desc.size() >= 7 && desc[2] == 1) {
            alt->endpoints.back().sampleRateControl = (desc[3] & 1u) != 0;
        }
    }
    if (alt)
        finishAlternate(*alt, result);

    for (auto& entry : result.controls) {
        if (entry.protocol != Protocol::uac2)
            continue;
        const auto range = std::find_if(associations.begin(), associations.end(), [&](Association association) {
            return entry.interface >= association.first && entry.interface < association.first + association.count;
        });
        if (range != associations.end()) {
            for (unsigned number = range->first; number < range->first + range->count; ++number)
                entry.streams.push_back(number);
        }
    }
    return result;
}

const Control* Topology::controlFor(const Stream& stream) const noexcept {
    const auto member = std::find_if(controls.begin(), controls.end(), [&](const Control& control) {
        return control.protocol == stream.protocol &&
               std::find(control.streams.begin(), control.streams.end(), stream.interface) != control.streams.end();
    });
    if (member != controls.end())
        return &*member;

    // Some single-function devices omit the IAD or UAC1 interface list.
    const Control* only = nullptr;
    for (const auto& candidate : controls) {
        if (candidate.protocol == stream.protocol && candidate.streams.empty()) {
            if (only != nullptr)
                return nullptr;
            only = &candidate;
        }
    }
    return only;
}

bool Stream::fits(unsigned rate, Speed speed) const noexcept {
    const auto ticks = ticksPerSecond(speed);
    if (data.intervalTicks > ticks / 125u || (feedback && feedback->intervalTicks > ticks / 4u))
        return false; // Bound audio transfer latency and feedback polling intervals.
    const auto numerator = static_cast<std::uint64_t>(rate) * data.intervalTicks;
    auto frames = (numerator + ticks - 1u) / ticks;
    // An asynchronous endpoint needs room for an extra frame if its clock is
    // slightly faster than the USB clock at an otherwise exactly divisible rate.
    if (!data.isInput() && feedback && numerator % ticks == 0)
        ++frames;
    if (frames * frameBytes() > data.packetBytes)
        return false;
    if (protocol == Protocol::uac2)
        return true; // The rate comes from the live Clock Source.
    // An adaptive OUT endpoint can recover its clock from the host stream.
    if (!data.sampleRateControl && (data.isInput() || data.synchronization != 2u))
        return (rates.size() == 1 && rates.front() == rate) || (minRate == rate && maxRate == rate);
    return std::find(rates.begin(), rates.end(), rate) != rates.end() ||
           (minRate != 0 && rate >= minRate && rate <= maxRate);
}

PacketClock::PacketClock(unsigned sampleRate, Speed speed, unsigned intervalTicks)
    : speed_(speed), ticks_(intervalTicks), nominal_(sampleRate * (framesUnit / ticksPerSecond(speed))),
      current_(nominal_) {}

bool PacketClock::applyFeedback(Bytes packet) noexcept {
    if (packet.size() != 3 && packet.size() != 4)
        return false;
    const auto valid = [&](std::int64_t rate) { return rate * 100 >= nominal_ * 94 && rate * 100 <= nominal_ * 106; };
    const auto raw = littleEndian(packet);
    auto rate = raw * (framesUnit / (packet.size() == 3 ? 16384 : 65536));
    if (speed_ == Speed::full && packet.size() == 4 && !valid(rate))
        rate = raw * (framesUnit / 16384); // A 10.14 value padded to four bytes.
    if (!valid(rate))
        return false;
    current_ += (rate - current_) / 8;
    return true;
}

unsigned PacketClock::nextFrames(unsigned maximum) noexcept {
    carry_ += current_ * ticks_;
    const auto count = static_cast<unsigned>(carry_ / framesUnit);
    carry_ %= framesUnit;
    return std::min(count, maximum);
}

float unpackPcm(const std::uint8_t* bytes, unsigned slotBytes) noexcept {
    const auto value = littleEndian(Bytes(bytes, slotBytes));
    const auto sign = std::uint64_t{1} << (slotBytes * 8u - 1u);
    const auto sample = static_cast<std::int64_t>(value) -
        ((value & sign) != 0 ? static_cast<std::int64_t>(2u * sign) : 0);
    return static_cast<float>(static_cast<double>(sample) / static_cast<double>(sign));
}

void packPcm(float sample, std::uint8_t* bytes, unsigned slotBytes, unsigned sampleBits) noexcept {
    const auto scale = std::int64_t{1} << (sampleBits - 1u);
    const auto clipped = std::isfinite(sample) ? std::clamp(static_cast<double>(sample), -1.0, 1.0) : 0.0;
    const auto quantized = std::min(scale - 1, static_cast<std::int64_t>(std::round(clipped * static_cast<double>(scale))));
    const auto value = static_cast<std::uint32_t>(quantized) << (slotBytes * 8u - sampleBits);
    for (unsigned i = 0; i < slotBytes; ++i)
        bytes[i] = static_cast<std::uint8_t>(value >> (i * 8u));
}

} // namespace wdf::android::usb::detail
