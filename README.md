# Android USB Audio

A C++20 library for app-local, direct USB audio on Android. It uses an Android USB Host permission grant to open a class-compliant interface, then streams PCM using Linux USB filesystem transfers. The app supplies its own audio processing callback. There are no third-party runtime dependencies.

## Supported USB streams

- USB Audio Class 1 and 2 PCM, with 16- to 32-bit signed samples in 2-, 3-, or 4-byte little-endian slots. Sample bits occupy the most significant bits of the slot. Audio callbacks use interleaved `float` samples.
- Full-speed and high-speed USB. Synchronous and adaptive OUT endpoints, and asynchronous OUT endpoints with a usable explicit feedback endpoint.
- Playback for output-only devices. The library prefers simultaneous input and output within the same AudioControl function. UAC2 duplex uses a shared clock source, including a source selected through a readable Clock Selector. If a compatible input cannot be opened, the library still tries playback and reports zero input channels.
- The requested sample rate defaults to 48 kHz. UAC1 rates come from the streaming descriptors. UAC2 clock rate is read or set and verified when opening the stream. The library operates at that rate without resampling.

Input-only devices, implicit feedback, multiple independent clock domains, UAC3, SuperSpeed streaming, and vendor-specific controls are outside this implementation. Firmware with independent input clocks can cause FIFO overrun or underrun. The kernel must support usbfs isochronous transfers and permit claiming the interface. Some phones keep their system audio driver bound or supply too little USB power for certain interfaces.

## Use in an Android app

Link the library from the app's NDK target.

```cmake
add_subdirectory(path/to/android-usb-audio)
target_link_libraries(my_native_app PRIVATE android_usb_audio)
```

Declare `android.hardware.usb.host` in the app manifest. For capture, request and obtain Android's `RECORD_AUDIO` permission before requesting USB device access. Use `UsbManager.requestPermission`, `UsbManager.openDevice`, `UsbDeviceConnection.getFileDescriptor`, and `UsbDeviceConnection.getRawDescriptors`. Pass the open descriptor and raw descriptor bytes through JNI. The library duplicates the descriptor. The caller keeps the `UsbDeviceConnection` alive, performs no competing requests while streaming, and closes it after the native device is closed. Handle USB attach and detach at the app level.

```cpp
#include <UsbAudioDevice.h>

wdf::android::usb::Device device;

// Called on an app control thread with the granted, open Java connection.
if (device.open(fd, rawDescriptors, 48000)) {
    const auto format = device.format();
    // Prepare the application callback for format and this block size.
    if (!device.start(64, processAudio, context))
        reportError(device.lastError());
} else {
    reportError(device.lastError());
}

// On detach or when leaving the audio screen, stop native I/O first.
device.close();
// Then close UsbDeviceConnection on Android.
```

The `AudioCallback` must be nonblocking and receives `nullptr` input for a playback-only connection. Stop and close wait for the callback thread and cancel outstanding USB transfers. For status displays, use `isRunning()`, `lastError()` and `statistics()`. The latter reports completed output frames, input frames lost to overflow, missing input frames filled with silence, and transfer or packet errors. A failed open or start returns an error, allowing the app to use its normal Android audio backend.

## Verification

Build with CMake 3.22 or newer and a C++20 compiler. On a Linux host, run the standalone tests without an Android SDK or audio device.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

For Android, use the NDK `android.toolchain.cmake` or the consuming application's Gradle build. Host tests cover configuration parsing, PCM conversion, sample pacing, and mocked usbfs streaming, including duplex feedback, playback-only devices, and cancellation after partial submission. Actual round-trip latency and stable buffer sizes need physical phone and interface tests. Test a dongle by playing a known signal, then test duplex with a USB interface using a cable loopback and hardware direct monitoring turned off.

## License

Copyright © 2026 Jegor van Opdorp. Licensed under the GNU Affero General Public License, version 3.
