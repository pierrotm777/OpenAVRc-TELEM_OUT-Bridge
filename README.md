# OpenAVRc-TELEM_OUT-Bridge

A lightweight telemetry bridge for **OpenAVRc**, designed to expose the radio's internal telemetry data through a clean output interface.

The goal of `TELEM_OUT` is simple:

> **Decode telemetry once inside OpenAVRc, then make the resulting telemetry data available to external devices without duplicating protocol-specific decoding.**

This project is currently used to bridge OpenAVRc telemetry to an **ESP32**, which can then forward the data to other systems such as **BLE applications**, external displays, loggers, or telemetry gateways.

---

## Overview

OpenAVRc already knows how to receive and decode telemetry from several RF protocols.

`TELEM_OUT` adds a second stage:

```text
Receiver / Sensors
        │
        ▼
RF Protocol
        │
        ▼
OpenAVRc telemetry decoder
        │
        ▼
telemetryData
        │
        ▼
TELEM_OUT
        │
        ▼
External device / ESP32
        │
        ▼
BLE / App / Logger / Display
```

The important point is that `TELEM_OUT` does **not** try to decode the RF protocol again.

The RF protocol and telemetry decoder remain responsible for interpreting the incoming telemetry. `TELEM_OUT` only reads the common OpenAVRc telemetry values and serializes them for external use.

---

## Why this project exists

Telemetry support in a radio firmware can easily become difficult to maintain when every output path implements its own decoding.

`TELEM_OUT` follows a simpler architecture:

- RF protocol code handles RF communication.
- Telemetry decoders update OpenAVRc's common telemetry structure.
- `TELEM_OUT` reads the already-decoded values.
- The external device only needs to understand the output protocol.

This avoids duplicated sensor mappings and keeps protocol-specific code where it belongs.

---

## Current use case

The current development setup is:

```text
OpenAVRc
   │
   │ TELEM_OUT
   ▼
ESP32-C3
   │
   │ CRSF telemetry stream
   ▼
BLE
   │
   ▼
Android Telemetry Viewer
```

The ESP32 acts mainly as a transport bridge.

Most telemetry frames are forwarded without changing their meaning.

This makes it possible to use OpenAVRc telemetry with modern external devices while keeping the original RF protocol handling inside the radio firmware.

---

## Output format

The current implementation uses **CRSF telemetry frames** as the external transport format.

Depending on the available OpenAVRc telemetry values, `TELEM_OUT` can generate standard CRSF telemetry frames such as:

- Link statistics
- Battery / voltage
- Current
- Capacity
- GPS position
- GPS altitude
- GPS speed
- Heading
- Variometer
- RPM
- Temperature
- Airspeed
- Cell voltage information
- Other telemetry values when a suitable common OpenAVRc field exists

Not every RF protocol provides every sensor.

`TELEM_OUT` can only transmit values that have already been decoded and stored in OpenAVRc's common telemetry data.

---

## Design principle

A very important rule in this project is:

> **Do not move protocol decoding into TELEM_OUT.**

For example:

```text
AFHDS2A RF frame
      │
      ▼
AFHDS2A / iBUS decoder
      │
      ▼
telemetryData.value.current
      │
      ▼
TELEM_OUT
      │
      ▼
CRSF Current / Battery frame
```

The same principle applies to HoTT, Hitec, FrSky, DEVO and other telemetry sources.

This keeps `TELEM_OUT` independent from the RF protocol used by the model.

---

## Building OpenAVRc with TELEM_OUT

Enable the feature in the OpenAVRc build options with:

```text
TELEM_OUT=YES
```

Example:

```text
EXT=FRSKY SERIAL_PROTOCOL=CRSF TELEM_OUT=YES
```

The exact build line depends on the radio hardware and the other OpenAVRc options being used.

---

## Development philosophy

This project tries to remain close to the existing OpenAVRc architecture.

In particular:

- avoid duplicated telemetry state,
- avoid decoding the same packet twice,
- keep RF timing-sensitive code as small as possible,
- reuse existing OpenAVRc telemetry fields,
- avoid unnecessary RAM usage,
- preserve the existing `pulses2MHz` / protocol workspace model where appropriate,
- keep protocol callbacks deterministic,
- prefer standard telemetry formats whenever possible.

On AVR targets, execution time and RAM usage matter, so even small changes inside RF callbacks should be considered carefully.

---

## Protocol independence

`TELEM_OUT` is not tied to one receiver protocol.

As long as a protocol updates the normal OpenAVRc telemetry structure, the same output code can be reused.

Conceptually:

```text
FrSky ───────┐
HoTT ────────┤
Hitec ───────┤
AFHDS2A/iBUS ├──► telemetryData ───► TELEM_OUT
DEVO ────────┤
Other ───────┘
```

This is one of the main advantages of the bridge architecture.

---

## ESP32 bridge

The external ESP32 can be kept relatively simple.

Its main jobs are typically:

1. Receive the telemetry stream from OpenAVRc.
2. Validate / forward the CRSF frames.
3. Transport them to the final consumer.

Possible destinations include:

- BLE applications
- USB tools
- Wi-Fi telemetry bridges
- loggers
- telemetry displays
- test and development tools

The ESP32 does not need to know how AFHDS2A, HoTT, Hitec or other RF telemetry protocols work.

---

## Status

`TELEM_OUT` is under active development.

The bridge is already usable, but telemetry coverage depends on:

- the RF protocol,
- the receiver,
- the connected sensors,
- the OpenAVRc decoder,
- and the availability of a suitable common telemetry field.

The project is intentionally developed incrementally so that each new telemetry value can be validated end-to-end:

```text
Sensor
  -> Receiver
  -> RF protocol
  -> OpenAVRc decoder
  -> telemetryData
  -> TELEM_OUT
  -> ESP32
  -> Application
```

---

## Debugging approach

When a telemetry value is missing, the preferred approach is to find where the chain stops.

For example:

```text
Sensor value present?
        │
        ▼
Received by RF protocol?
        │
        ▼
Decoded into telemetryData?
        │
        ▼
Generated by TELEM_OUT?
        │
        ▼
Received by ESP32?
        │
        ▼
Displayed by the application?
```

This makes debugging much easier than adding protocol-specific workarounds at the end of the chain.

---

## Compatibility

The project is intended for use with OpenAVRc and its existing telemetry architecture.

Current development is focused on AVR-based OpenAVRc targets and ESP32-based external bridges.

Because OpenAVRc is timing-sensitive, changes to RF protocol callbacks should ideally be checked with the radio's statistics / timing tools before being merged.

---

## Project goals

The long-term goals are:

- provide a clean telemetry output layer for OpenAVRc,
- support as many common telemetry values as possible,
- keep RF protocol code independent from the external transport,
- make it easy to connect OpenAVRc to modern devices,
- avoid unnecessary protocol-specific bridges,
- keep the implementation small enough for AVR targets.

---

## Contributions

Testing is especially useful with different:

- RF protocols,
- receivers,
- sensor combinations,
- OpenAVRc boards,
- ESP32 bridge configurations,
- telemetry applications.

When contributing, please try to preserve the separation between:

```text
RF reception
Telemetry decoding
Common telemetry storage
TELEM_OUT serialization
External transport
```

This separation is what keeps the project maintainable.

---

## License

This project is based on and intended to work with **OpenAVRc**.

Please refer to the OpenAVRc project for the applicable license terms and original copyright information.

---

## Name

**OpenAVRc-TELEM_OUT-Bridge**

The name reflects the purpose of the project: a bridge between OpenAVRc's internal telemetry system and external telemetry consumers.
