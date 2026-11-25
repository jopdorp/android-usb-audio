#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace wdf::android::usb::detail {

using Bytes = std::span<const std::uint8_t>;

enum class Speed { full, high };
enum class Protocol { uac1, uac2 };

struct Endpoint {
    unsigned address = 0;
    unsigned packetBytes = 0;
    unsigned intervalTicks = 0;
    unsigned synchronization = 0;
    unsigned usage = 0;
    unsigned syncAddress = 0;
    bool sampleRateControl = false;

    bool isInput() const noexcept { return (address & 0x80u) != 0; }
};

struct Clock {
    unsigned id = 0;
    bool writableRate = false;
    std::vector<unsigned> selectorSources;
};

struct Control {
    unsigned interface = 0;
    Protocol protocol = Protocol::uac1;
    std::vector<unsigned> streams;
    std::vector<Clock> clocks;
    std::vector<std::pair<unsigned, unsigned>> terminalClocks;
};

struct Stream {
    unsigned interface = 0;
    unsigned alternate = 0;
    Protocol protocol = Protocol::uac1;
    unsigned terminal = 0;
    unsigned channels = 0;
    unsigned slotBytes = 0;
    unsigned sampleBits = 0;
    std::vector<unsigned> rates;
    unsigned minRate = 0;
    unsigned maxRate = 0;
    Endpoint data;
    std::optional<Endpoint> feedback;

    unsigned frameBytes() const noexcept { return channels * slotBytes; }
    bool fits(unsigned rate, Speed speed) const noexcept;
};

struct Topology {
    std::vector<Control> controls;
    std::vector<Stream> streams;

    const Control* controlFor(const Stream& stream) const noexcept;
};

unsigned littleEndian(Bytes bytes) noexcept;
unsigned ticksPerSecond(Speed speed) noexcept;
Bytes findConfiguration(Bytes raw, unsigned value) noexcept;
std::optional<Topology> parseConfiguration(Bytes config, Speed speed);

// Explicit USB 2.0 feedback expresses sample frames per USB frame/microframe.
// The packetizer preserves fractional frames across consecutive packets.
class PacketClock {
public:
    PacketClock(unsigned sampleRate, Speed speed, unsigned intervalTicks);
    bool applyFeedback(Bytes packet) noexcept;
    unsigned nextFrames(unsigned maximum) noexcept;

private:
    Speed speed_;
    unsigned ticks_;
    std::int64_t nominal_;
    std::int64_t current_;
    std::int64_t carry_ = 0;
};

float unpackPcm(const std::uint8_t* bytes, unsigned slotBytes) noexcept;
void packPcm(float sample, std::uint8_t* bytes, unsigned slotBytes, unsigned sampleBits) noexcept;

} // namespace wdf::android::usb::detail
