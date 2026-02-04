## Open Theremin V3.5 with MIDI interface control software V2.9 for Arduino UNO R4

### UNO R4 only
- This repository now targets `Arduino UNO R4` only.
- Audio tick generation requires UNO R4 hardware timer resources (`FspTimer`).
- Audio tick rate is configurable in `Open_Theremin_V3/build.h` (`OT_AUDIO_TICK_HZ`).
- Audio tick rate can also be changed at runtime via `ihSetAudioTickHz(hz)` (range in `build.h`).
- Simple preset helper is available via `OT_AUDIO_RATE_PRESET` in `Open_Theremin_V3/build.h`.
- Optional SPI DMA backend can be enabled with `OT_USE_DMA` in `Open_Theremin_V3/build.h`.
- DMA backend now includes strict transfer-open checks and per-frame timeout via `OT_DMA_TRANSFER_TIMEOUT_US`.
- Live-safe default: DMA frame timeout does not hard-stop audio (`OT_DMA_HARDFAIL=0`), it flags LED error and continues.
- Central LED error indicator is active (`setErrorIndicator` / `fatalErrorLoop`): blink code `1`=DMA, `2`=Timer, `3`=Audio-rate setup.
- MIDI-IN control channel is fixed via `OT_MIDI_IN_CHANNEL` in `Open_Theremin_V3/build.h`.
- MIDI transport is selected via `OT_MIDI_NATIVE_USB` in `Open_Theremin_V3/build.h` (or `build_flags` in `platformio.ini`).
- Calibration frequency measurements use interrupt edge counting.


Based on Arduino UNO Software for the [Open.Theremin version 3.0  Copyright (C) 2010-2016 by Urs Gaudenz](https://github.com/GaudiLabs/OpenTheremin_V3)
 and [MrDHam's fork](https://github.com/MrDham/OpenTheremin_V3_with_MIDI) 

This Open Theremin V3.5 with MIDI, since version V2.6, also takes into account 
Changes added in Open.Theremin version 3.1 (all by @Theremingenieur):

    Fix a wavetable addressing issue (found by @miguelfreitas)
    Use the Arduino's hardware SPI to control the DACS and use the Latch signal to reduce audio jitter
    Improve the register switch to transpose by clean octaves and keep the tone spacing and pitch tuning consistent
    Improve the volume response to give a smoother start and wider dynamics (*)

(*) This relies on a recent gcc compiler version. Make sure to compile it with the Arduino IDE >= 1.8.10

Pitch Bend Range choice is also extended (Allows 4 octaves Bend) 

MIDI CC from Pitch antenna (Rod) have 14 Bit resolution when applicable. Example of application: you can create a software synth controled by CC7 for volume and CC16 (MSB) - CC48 (LSB) for pitch without using Note On/Off and Pitch Bend messages. 

Urs also made a very clear presentation of the MIDI feature on his website: http://www.gaudi.ch/OpenTheremin/index.php?option=com_content&view=article&id=200&Itemid=121, many thanks !  

### Don't click on the files!
Click on the "clone or download" Button to the right. Then unpack the archive.

### Open Source Theremin based on the Arduino Platform + MIDI communication

Open.Theremin is an arduino shield to build the legendary music instrument invented by Leon Theremin back in 1920. The theremin is played with two antennas, one to control the pitch and one for volume. The electronic shield with two ports to connect those antennas comprises two heterodyne oscillators to measure the distance of the hand to the antenna when playing the instrument. The resulting signal is fed into the arduino. After linearization and filtering the arduino generates the instruments sound that is then played through a high quality digital analog audio converter on the board. The characteristics of the sound can be determined by a wave table on the arduino.

For more info on the open source project and on availability of ready made shield see:

http://www.gaudi.ch/OpenTheremin/

This githup repository provides the code to add a MIDI interface to the Open Theremin V3 so as you can connect it to your favourite synthesizer. 

### Installation
1. Open up the Arduino IDE
2. Open the File "Open_Theremin_V3.ino"
3. Select the correct arduino board from Tools -> Board (`Arduino UNO R4 Minima` or `Arduino UNO R4 WiFi`).
4. Select the correct USB port on Tools -> Serial Port.
5. Upload the code by clicking on the upload button.
6. Default MIDI transport is USB serial (MIDI byte stream at `OT_MIDI_SERIAL_BAUD`), use a USB-serial MIDI bridge/router on host side.
7. Experimental native USB-MIDI build targets are available: `uno_r4_minima_usb_midi`, `uno_r4_wifi_usb_midi`.
8. Native USB-MIDI envs use a project-local patched UNO R4 core in `third_party/framework-arduinorenesas-uno`.

### MIDI-IN controls (USB serial stream + native USB-MIDI)
The following controls are handled on `OT_MIDI_IN_CHANNEL`:
- Program Change `0..11`: recall one of 12 tone presets.
- CC `20`: Mute toggle (`>=64` mute, `<64` normal).
- CC `21`: Panic (All Notes Off).
- CC `22`: Legato (`>=64` on).
- CC `23`: Pitch Bend enable (`>=64` on).
- CC `24`: Pitch Bend range (mapped to `1,2,4,5,7,12,24,48`).
- CC `25`: Volume trigger (`0..127`).
- CC `26`: Wavetable select (`0..13` mapped from CC range).
- CC `27`: Audio tick preset (`31.25k / 40k / 48k`).
- CC `28`: Wavetable morph on/off (`>=64` on).
- CC `29`: Pitch tilt filter on/off (`>=64` on).
- CC `30`: Soft clip on/off (`>=64` on).
- CC `31`: Wavetable morph speed (`1..16` internal range).
- CC `32`: Pitch tilt amount (`0..255` internal wet max).
- CC `33`: Soft clip drive (higher CC = stronger drive).
- CC `34`: Subtle vibrato+jitter on/off (`>=64` on).
- Calibration remote combo: CC `102` value `42`, then within 2s CC `103` value `99`.

Wavetable MIDI reference (for CC `26` / Timbre pot):
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

Tone preset reference (for Program Change `0..11`):
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
Program Change values above `11` are ignored.

When selecting a timbre preset (MIDI Program Change or waveform preset via pot), audio feature defaults are restored.

Additional timbres were added: Triangle-Vocal, Soft-Saw, Clarinet, and Cello.
Wavetable order is arranged roughly from warm/dark to bright/edgy across indices `0..13`.

Note: native USB-MIDI depends on UNO R4 core support (`CFG_TUD_MIDI=1` + MIDI interface descriptor availability).

### Added and removed compare to Open Theremin V3. 
Serial communication implemented for program monitoring purpose was removed (Particularly during calibration).
If you need to monitor calibration for antenna problem fixing, please use original master branch from 
https://github.com/GaudiLabs/OpenTheremin_V3. 

Serial port is used to send MIDI messages now. 

### How does it work ? 

The MIDI open theremin generates NOTE ON/OFF messages and  Continuous Controler changes (MIDI CC) depending on settings and hands' position next to antennas. 


MIDI CC: 

It is possible to assign independant MIDI CCs to the PITCH ANTENNA (ROD) and to the VOLUME ANTENNA (LOOP).  

NOTE ON/OFF: 

In MIDI standard NOTE ON/OFF messages have a NOTE NUMBER and a VELOCITY. 

Let's consider a Fade-in / Picth Variation / Fade-out sequence (I use right handed convention): 

1. Fade-In

   When left hand moves away from VOLUME ANTENNA (LOOP) and volume crosses a settable threshold (Volume trigger), a NOTE ON is generated. VELOCITY depends on how fast left hand is moving. Right hand's position next to PITCH ANTENNA (ROD) determines the starting NOTE NUMBER. 


2. Pitch variation

   When right hand moves next to PITCH ANTENNA (ROD), PITCH BEND messages are generated (if activated) to reach exact pitch as long as pitch bend range will do.  Beyond, a new NOTE ON followed by a NOTE OFF for the previous note are generated if legato mode is activated. Pitch bend range can be configured (1, 2, 4, 5, 7, 12, 24 or 48 semitones) to align with synth's maximum capabilities. 

3. Fade-Out

   When left hand moves close to VOLUME ANTENNA (LOOP) and volume goes under Volume trigger threshold, a NOTE OFF is generated to mute the playing note. 

  
 SETTINGS:
 
 "Register" pot becomes "Selected Parameter" pot and have 8 positions. 
  "Timbre" pot becomes "Parameter's Value" and have a variable number of positions depending on selected parameter: 
 
 1. Register: 3 positions (-1 Octave, center, +1 Octave) as in original Open Theremin V3 (version V3.1)
 2. Timbre: 8 positions as in original Open Theremin V3
 3. Channel: 16 positions (channel 1 to 16)
 4. Rod antenna mode: 4 positions 
     (Legato off/Pitch Bend off, Legato off/Pitch Bend on, Legato on/Pitch Bend off, Legato on/Pitch Bend on)
 5. Pitch bend range: 8 positions (1, 2, 4, 5, 7, 12, 24, 48 Semitones). 
     For classical glissando and in order to have same note on audio and MIDI, use exactly same pitch bend range on your synth. 
     Maximum setting possible is recomended.
 6. Volume trigger / Velocity sensitivity (how fast moves the volume loop's hand): 128 positions (0 to 127)
 7. Rod antenna MIDI CC: 8 positions 
    (None, 8-Balance, 10-Pan, 16-MSB/48-LSB-GeneralPurpose-1, 17-MSB/49-LSB-GeneralPurpose-2, 18-GeneralPurpose-3, 19-GeneralPurpose-4, 74-cutoff) 
    
    For 14 Bit CC messages, MSB and LSB are always sent together and in the following order: MSB (1st), LSB (2nd) as per MIDI 1.0 Standard. 
    The receiver can bufferize MSB to synchronize it with the LSB. 
    
 8. Loop antenna MIDI CC: 8 positions 
    (1-Modulation, 7-Volume, 11-Expression, 71-Resonnance, 74-Cutoff, 91-Reverb, 93-Chorus, 95-Phaser)

Select a Parameter and move "Parameter's Value" to change corresponding setting. 

While you rotate the pots, both LEDs toggles (OFF/ON) every steps to give you some angular feedback before going back to PLAY/MUTE Status.

The picture at https://github.com/MrDham/OpenTheremin_V3_with_MIDI/blob/master/MIDI%20Open%20Theremin%20V3%20HMI.bmp gives an example of possible HMI: on "Value" pot, red lines have 4 positions, grey lines have 5 positions and yellow lines have 8 positions. On "Parameter" pot you see coloured lines indicating which colour to follow for the "Value" pot. 

The Quick Guide at https://github.com/MrDham/OpenTheremin_V3_with_MIDI/blob/master/Quick%20guide%20open%20theremin%20midi.bmp works well with this HMI. Print it on a portrait A4 sheet of paper, plastify it and take it with your theremin everywhere you go... 

The volume trigger can be configured so as we have some volume at note attack on percussive sounds. 
The volume trigger setting is also used to set sensitivity for velocity (how fast left hand is moving when note is triggered). 
Volume trigger = 127 (Maximum) won't generate any NOTE ON. It can be used to generate MIDI CC only.

Manipulation of "Rod antenna MIDI CC" and "Loop antenna MIDI CC" is not error proof. MIDI newbies should be advised to change their value in MUTE mode. 

 
Default configuration is: Register = Center, Timbre = 1st Waveform, Channel = MIDI Channel 1, Rod antenna mode = Legato on/Pitch Bend on, Pitch bend range = 2 Semitones, Volume trigger = 0, Rod antenna MIDI CC = None, Loop antenna MIDI CC = 7-Volume. 


MUTE BUTTON: 

Sends ALL NOTE OFF on selected channel and stay in mute until it is pushed again.  

AUDIO: 

Audio processing from antennas to output jack, including volume and pitch pots, LEDs and button functions, is exactly the same as in open theremin V3.  You can play the Audio and the MIDI side by side. 

CALIBRATION:

This device runs normal calibration of antennas after pushing button for 3 seconds as per initial project

### What can I do to get a theremin like glissando?

Activate picth bend and set pitch bend range of the theremin with a high value (12 semitones or 24 semitones).
Set pitch bend range of the synth with the same value. 


### If I do not trigger with the volume hand it also seems to trigger a new tone with the pitch antenna. Guess this is how MIDI works.

When legato mode is activated, if you trigger a note (with volume loop) and go in one direction (with pitch antenna) a new note will be triggered at the limit of pitch bend range. 

Legato mode is used as a workaround for limitation of Pitch Bend range on some synths (e.g. max 12 semitones pitch bend). 

### With Legato Mode = ON and Pitch Bend Mode = OFF, the notes generated don't seem to be in a given scale. For example I can't play in C major if I select Pitch Bend Range = 2.
Effectively, according to the sequence described above (Fade-in / Picth Variation / Fade-out), the first note played influences the next notes played: they will always be distant of N * Pitch Bend Range. For example, with Pitch Bend Range = 2, if you start with a C and move the right hand, you can only play C, D, E, F#, G#, A#. This is as designed. 

If you want to play in a given scale you need to set Legato Mode = ON, Pitch Bend Mode = OFF, Pitch Bend Range = 1 and to use a synth with a "force to scale" capacity. Then select the expected scale and play. Out of scale notes will be replaced by the closest one in the scale. 

### What is the problem if MIDI messages appear to be messy and inconsistent resulting in strange note played, strange synth parameter changes, ... ?
Most probable cause is host-side USB-serial MIDI routing configuration, wrong selected device port, or a synth-side MIDI channel/range mismatch.

### Tweakable parameters (in application.cpp):
Changing this to your taste may require some test and trial. 

"#define VELOCITY_SENS  9" -> How easy it is to reach highest velocity (127). Something betwen 5 and 12.  

"#define PLAYER_ACCURACY  819"  -> Pitch accuracy of player. Tolerance on note center for changing notes when playing legato. From 0 (very accurate players) to 2048 (may generate note toggling). 

### NEED SUPPORT ?
Please log bugs, requests and questions at https://github.com/MrDham/OpenTheremin_V3_with_MIDI/issues

### Any questions about MIDI and theremins ?
The answer is probably there: https://app.box.com/s/s3yx1ro1v8ay061626wy09vmbo7do23q 


### LICENSE
Original project, Open Theremin, was written by Urs Gaudenz, GaudiLabs, in 2016
GNU license. This Project inherits this 2016 GNU License. 

 Check LICENSE file for more information

All text above must be included in any redistribution
