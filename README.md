# ESP32-Acoustic-Drone-Detector
Portable ESP32 acoustic drone-warning prototype using two microphones, FFT analysis and vibration alerts.


A portable acoustic warning prototype that uses an ESP32, two INMP441 digital microphones, an OLED display and a vibration motor to detect persistent drone-like sound patterns.

## How It Works

The system does not react to loudness alone. It searches for a possible motor or propeller base frequency and checks for related harmonic frequencies.

For example, if the base frequency is 200 Hz, the detector looks for matching frequencies near 400, 600, 800, 1000 and 1200 Hz.

A vibration warning is activated only when:

- Sound is above the calibrated background threshold
- A valid base frequency is detected
- At least three expected harmonics are present
- Both microphones receive the sound
- The spectral shape is more complex than a pure test tone
- The frequency has controlled motor-like variation
- At least 65% of recent analysis windows pass
- The pattern continues for at least five seconds
- Confidence reaches at least 85%

## Hardware

- ESP32
- Two INMP441 I2S microphones
- SSD1306 128×64 OLED display
- Vibration motor
- PN2222 transistor
- USB power bank
- 3D-printed enclosure

## Signal Processing

- 16 kHz stereo audio sampling
- 1,024-point FFT with Hann window
- 70–800 Hz base-frequency search
- Harmonic analysis through 6 kHz
- Adaptive background calibration
- Spectral-distribution analysis
- Frequency and amplitude modulation tracking
- Two-microphone agreement
- Rolling evidence and confidence scoring
- Rough LEFT, CENTER and RIGHT indication

## Display Information

The OLED shows:

- Detection state
- Confidence
- Current sound energy
- Adaptive threshold
- Dominant live frequency
- Rough direction
- USB power status

## Development Status

This is an engineering MVP and acoustic warning prototype. It does not identify a drone model, calculate an exact location or guarantee drone detection.

The current development stage focuses on:

- Collecting labelled outdoor drone recordings
- Testing against fans, traffic, music, tools and other difficult sounds
- Measuring detection and false-alert rates
- Improving the enclosure and microphone placement
- Refining detection thresholds using real field data

## Project Files

- `firmware/`: ESP32 Arduino firmware
- `documentation/`: Engineering and detection-logic guide
- `images/`: Prototype and wiring images

## Author

Your Name  John

Email: jtiongco2601@gmail.com
