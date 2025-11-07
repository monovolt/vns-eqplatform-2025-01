# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino-based equatorial tracking platform (vns-eqplatform) designed for astronomy applications. The platform uses a stepper motor with TMC2209 driver to follow celestial objects across the sky at a precise tracking speed. It includes IR remote control, OLED display feedback, piezo audio cues, and optional MPU-9250 gyroscope/accelerometer for automatic leveling and position detection.

**Language**: Arduino C++ (.ino files)
**Hardware**: Arduino board, TMC2209 stepper driver, OLED SSD1306 display, IR receiver, MPU-9250 sensor (optional), stepper motor

## Repository Structure

- **eqplatform-01/eqplatform-01.ino** - Main active codebase with full features
- **eqplatform-old/eqplatform-old.ino** - Older version without gyroscope/accelerometer support
- **lg_melody/lg_melody.ino** - Test sketch for audio melodies
- **README.md** - Korean documentation with pin connections and button mappings

## Development Environment

### Building and Uploading
Use the Arduino IDE or PlatformIO to compile and upload:
- Open the .ino file in Arduino IDE
- Select the appropriate Arduino board (likely Arduino Uno or Mega based on pin usage)
- Install required libraries (see below)
- Click Upload to compile and flash to the board

### Required Libraries
Install these via Arduino Library Manager:
- **Wire.h** - I2C communication (built-in)
- **IRremote** - IR receiver library
- **Adafruit_GFX** - Graphics library for OLED
- **Adafruit_SSD1306** - OLED display driver
- **TMCStepper** - TMC2209 stepper driver control
- **MPU9250_asukiaaa** - MPU-9250 gyroscope/accelerometer sensor

### Pin Connections (eqplatform-01)
Refer to README.md for TMC2226 UART connections. Key pins:
- Pins 0-1: Serial1 (TX/RX) for TMC2209 UART
- Pin 2: Motor enable (LOW=enabled)
- Pin 3: Step signal
- Pin 4: Direction control
- Pins 5-9: Physical buttons (INPUT_PULLUP)
- Pin 10: Piezo speaker
- Pin 11: Red LED (power bank keep-alive)
- Pin 12: IR receiver

## Code Architecture

### State Machine Design
The platform operates in distinct states tracked by `currentStatus` string:
- **"Welcome.."** - Initial/centered position
- **"Start Rdy"** - At right end, ready to begin tracking
- **"Tracking.."** - Actively tracking celestial motion (slow leftward movement)
- **"End."** - Reached left end, tracking complete
- **"Stopped"** - User-initiated stop
- **"Mov B Pos"** - Moving back to start position (fast rightward)
- **"Mov E Pos"** - Moving to end position (fast leftward)
- **"Leveling>" / "Leveling<"** - Auto-leveling in progress

State transitions occur via button presses (checkButtons) or IR remote commands (checkIR).

### Motor Control Strategy
Two-speed operation controlled by `tracking_delay` variable:
- **Tracking mode**: Uses `tracking_delay` (typically 5300-11365 microseconds) for slow celestial tracking in clockwise direction
- **Fast positioning**: Uses `MAX_DELAY` (300 microseconds) for rapid counterclockwise movement back to start position
- Direction controlled by `dirPin`: LOW=counterclockwise (fast return to start), HIGH=clockwise (slow tracking)
- When `moveRight=true`: Fast counterclockwise movement (dirPin=LOW) back to start position
- When `moveRight=false`: Slow clockwise tracking (dirPin=HIGH) following celestial motion
- Steps generated in loop() via `stepMotors()` when `motorRunning=true`

### Sensor Integration (Optional)
The MPU-9250 gyroscope provides automatic features when `isSensorOn=true`:
- **Auto-leveling**: `startLeveling()` moves platform to horizontal (roll ≈ 0°)
- **Limit detection**: `checkLeftLimit()` and `checkRightLimit()` stop at ±6.5° tilt (configurable via `LIMIT_ANGLE`)
- **Signal filtering**: Multi-stage filter (`filterValue()`) with moving average and jump detection to handle sensor noise
- Sensor readings updated every 200ms to reduce processing overhead
- **Upside-down mounting compensation**: The sensor is mounted upside-down (180° rotation), so the code applies a 180° offset correction plus left/right reversal to compensate

The sensor is toggled on/off via IR remote button 8 (command 0x15), with audio feedback (beep for ON, low beep for OFF).

### IR Remote Command Mapping
Critical commands (from checkIR function):
- 0x45 (69): End position reached
- 0x46 (70): Stop
- 0x47 (71): Start position ready
- 0x40 (64): Start tracking
- 0x44 (68) / 0x43 (67): Decrease/increase speed by 500
- 0x18 (24) / 0x52 (82): Fine speed adjust ±50
- 0x19 (25): Save current tracking speed
- 0x16 (22): Center and level
- 0x1C (28): Resume with saved speed
- 0x07 (7): Toggle sound on/off (with short beep confirmation when enabled)
- 0x15 (21): Toggle gyroscope sensor on/off (with audio feedback)
- 0x5A (90) / 0x08 (8): Fast move right/left

### Display Updates
OLED display shows:
- Current state/status (2x size text)
- Current delay value ("D : XXXXX")
- Saved memory delay
- Sensor and Sound on/off status ("Lv.Snsr: On/Off Snd: On/Off")
- Roll angle (left/right tilt in degrees)
- Pitch angle (forward/back tilt in degrees)

Display refreshes every 800ms when sensor is active to prevent slowdown. When sensor or sound state changes, display updates immediately.

## Common Modifications

### Adjusting Tracking Speed
The tracking speed is controlled by `default_delay` (eqplatform-01.ino:37). Current value is 11365 microseconds. This represents the time between motor steps during celestial tracking. Lower values = faster tracking. This can be adjusted in real-time via IR remote and saved with button 0.

### Changing Tilt Limits
Modify `LIMIT_ANGLE` constant (eqplatform-01.ino:10) to change when the platform stops at extreme positions. Current value is 6.5 degrees (±6.5°). This narrow range prevents the platform from tilting too far during operation.

### Audio Feedback
Multiple melodies defined for different events:
- `starMelody`: Start position ready
- `lgMelody`: End position reached (LG shutdown sound)
- `welcomeMelody`: Welcome/centered state
- `stopMelodyA/B`: Stop commands
- `levelingMelodyA`: Leveling started
- `trackingMelodyB`: Tracking started

All audio functions (`playMelody()`, `playBeep()`, `playBeepLow()`) check the `isSoundOn` flag. When sound is disabled via IR button 7, all audio is muted. Modify arrays starting around line 82 to customize audio feedback.

### Sensor Filtering
If sensor readings are too noisy or too slow to respond:
- `FILTER_SIZE` (line 170): Moving average window size (default 10)
- `JUMP_THRESHOLD` (line 171): Ignore sudden changes larger than this (default 5.0°)
- `MAX_JUMP_COUNT` (line 174): How many consecutive jumps before accepting change (default 3)

## Important Notes

- The code uses Serial1 for TMC2209 UART communication, not the main Serial port
- TMC2209 is configured for 8 microsteps with spreadCycle mode (not stealthChop)
- The hold_multiplier is set to 1.0 (100% hold current) to prevent power bank auto-shutoff
- **Motor direction logic**: The platform tracks by rotating clockwise (slow) and returns counterclockwise (fast). This matches Earth's rotation when the platform is oriented toward Polaris (North Star)
- **MPU-9250 sensor is mounted upside-down**: Code compensates with 180° rotation correction (lines 311-313) plus left/right reversal to get correct orientation readings
- Korean comments throughout the code provide additional context
