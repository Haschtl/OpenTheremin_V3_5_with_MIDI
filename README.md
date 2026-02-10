# OpenTheremin V3.5 (UNO R4, MIDI Edition)

This fork targets **Arduino UNO R4 Minima / UNO R4 WiFi** and focuses on:
- stable audio timing (hardware timer, optional DMA),
- expanded timbre engine (tables + PolyBLEP),
- practical MIDI control (Program Change + CC),
- live-friendly tone presets.

## Quick Start

### 1) Hardware
- Target board: **UNO R4 Minima** or **UNO R4 WiFi**
- Shield/wiring as defined in `Open_Theremin_V3/pins.h`
- Note: Pitch capture and button pins are swapped (`D6` = pitch capture, `D8` = button) to use a GPT32 channel for pitch.

### 2) Build (PlatformIO)
Default firmware:
- `uno_r4_minima` or `uno_r4_wifi`

Experimental native USB-MIDI:
- `uno_r4_minima_usb_midi` or `uno_r4_wifi_usb_midi`
- Uses patched local core in `third_party/framework-arduinorenesas-uno`

### 3) Flash
- Upload over USB as usual from PlatformIO/Arduino IDE.

## Relevant Files

- Firmware: `Open_Theremin_V3/`
- Main config: `Open_Theremin_V3/build.h`
- Audio engine ISR: `Open_Theremin_V3/ihandlers.cpp`
- App + MIDI + UI logic: `Open_Theremin_V3/application.cpp`
- PlatformIO config: `platformio.ini`

## Key Features

- UNO R4-only port (AVR/UNO R3 removed)
- Hardware timer audio tick with runtime rate selection
- Optional SPI DMA backend
- 14 timbres (12 tables + 2 procedural PolyBLEP)
- Biquad tilt filter, soft clip, morphing, subtle vibrato/jitter
- MIDI-In control with Program Change tone presets
- LED error indication (blink codes)

## Build Configuration (`build.h`)

Common controls:
- `OT_AUDIO_TICK_HZ`, `OT_AUDIO_TICK_HZ_MIN`, `OT_AUDIO_TICK_HZ_MAX`
- `OT_USE_DMA`
- `OT_DMA_TRANSFER_TIMEOUT_US`
- `OT_DMA_HARDFAIL` (`0` live-safe, `1` hard stop)
- `OT_MIDI_IN_CHANNEL` (default startup MIDI channel)
- `OT_MIDI_NATIVE_USB` (`0` serial MIDI stream, `1` native USB-MIDI)

Audio/timbre defaults:
- Morph / Tilt / Softclip / Vibrato enable defaults
- Biquad tilt tuning (Q, min/max cutoff)
- Vibrato/jitter depth settings

## MIDI-In Reference

All controls are processed on `OT_MIDI_IN_CHANNEL`.

### Program Change
- `PC 0..11`: load one of 12 tone presets
- Values > `11` are ignored

### CC
- `20`: Mute toggle (`>=64` mute)
- `21`: Panic (All Notes Off)
- `22`: Legato on/off
- `23`: Pitch-bend on/off
- `24`: Pitch-bend range (`1,2,4,5,7,12,24,48`)
- `25`: Volume trigger (`0..127`)
- `26`: Wavetable select (`0..13`)
- `27`: Audio-rate preset (`31.25k / 40k / 48k`)
- `28`: Morph on/off
- `29`: Tilt filter on/off
- `30`: Softclip on/off
- `31`: Morph speed
- `32`: Tilt amount
- `33`: Softclip drive
- `34`: Vibrato/jitter on/off
- `36`: Set active MIDI channel (`0..127` -> channel `1..16`)
- `102` + `103`: Remote calibration arm/confirm sequence

## MIDI-Out Reference

Outgoing MIDI is sent on the current active channel (`midi_channel`), which defaults to
`OT_MIDI_IN_CHANNEL` and can be changed/persisted via CC `36`.

### Messages sent
- `Note On` / `Note Off`:
  - `0x90` note on with velocity when volume crosses trigger
  - `0x90` with velocity `0` for note off
- `Pitch Bend`:
  - `0xE0` with 14-bit bend value (center = 8192)
  - follows selected bend range (`1,2,4,5,7,12,24,48`)
- `Loop CC` (volume hand):
  - configurable 7-bit CC output (`loop_midi_cc`, 0..127)
- `Rod CC` (pitch hand):
  - configurable CC output (`rod_midi_cc`)
  - optional 14-bit mode with LSB companion (`rod_midi_cc_lo`)
  - when 14-bit is active, order is always **MSB first, then LSB**
- `All Notes Off`:
  - `CC 123` is sent on panic/mute/channel switch paths

### Typical behavior
- Note generation is controlled by volume trigger and current mode.
- In legato mode, pitch movement can retrigger note transitions while maintaining continuous play feel.
- If pitch bend is disabled, bend output is held at center.

## Tone Presets (PC 0..11)

- `0` Classic Sing
- `1` Cello Air
- `2` Vocal Lead
- `3` Soft Expressive
- `4` Clarinet Glide
- `5` Bright Classic
- `6` Soft-Saw Warm
- `7` Phoenix Edge
- `8` Modern Mono
- `9` Razor Solo
- `10` PolyBLEP Saw
- `11` PolyBLEP Pulse

## Wavetable Reference (CC26 / Timbre)

- `0` Classic Sine (`sine_table`)
- `1` Warm Sine A (`sine_table6`)
- `2` Warm Sine B (`sine_table5`)
- `3` Cello (`sine_table12`)
- `4` Triangle-Vocal (`sine_table9`)
- `5` Sinus Variant A (`sine_table3`)
- `6` Sinus Variant B (`sine_table4`)
- `7` Clarinet-like (`sine_table11`)
- `8` Bright Sinus A (`sine_table8`)
- `9` Bright Sinus B (`sine_table7`)
- `10` Soft-Saw (`sine_table10`)
- `11` Phoenix / Brightest (`sine_table2`)
- `12` PolyBLEP Saw (procedural)
- `13` PolyBLEP Pulse 50% (procedural)

## Pot UI Behavior

Parameter pot (`REGISTER_SELECT_POT`) is mode-dependent:
- Audio-active (`NORMAL` mode): 4 segments
- `0` Register (transpose)
- `1` Tone preset (0..11)
- `2` Timbre/wavetable (0..13)
- `3` Master out gain (10%..100%)
- Standby (`MUTE` mode): 5 segments
- `0` Rod mode (legato / pitch-bend mode)
- `1` Pitch-bend range
- `2` Trigger volume/velocity
- `3` Rod CC mode
- `4` Loop CC mode

Value pot (`WAVE_SELECT_POT`) sets the currently selected segment value.

Notes:
- MIDI channel can be changed at runtime via CC `36`.
- MIDI channel selected via CC `36` is persisted in EEPROM.
- Preset/timbre selection restores audio-feature defaults.

## LED Error Indicator

Central API: `setErrorIndicator(...)` / `fatalErrorLoop(...)`

Blink codes:
- `1` DMA error
- `2` Timer error
- `3` Audio-rate setup error

## DMA Behavior

- `OT_DMA_HARDFAIL=0` (default): DMA frame errors do not hard-stop audio; LED error is set.
- `OT_DMA_HARDFAIL=1`: DMA errors trigger fatal hard-stop loop.

## Credits

Based on:
- GaudiLabs OpenTheremin V3
- MIDI extension work from the MrDham fork

This repository contains a UNO R4-focused modernization of audio + MIDI paths.
