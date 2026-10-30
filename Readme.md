# Hob2Hood Kettle

Links an electric kettle with a Hob2Hood-capable kitchen hood. While the kettle is boiling, an Arduino UNO detects its current draw and sends Hob2Hood infrared commands to the hood: the light comes on and the fan speeds up step by step to pick up the steam.

Author: 1Pats@oto.lv, March 2024. Developed with help from Armin Joachimsmeyer (ArminJo), co-author of the IRremote library, whose `ReceiveAndSendHob2Hood` example the IR part is based on.

## How it works

1. A ZMCT103C current sensor measures the kettle socket. The sketch samples it for ~16 ms (≈ ¾ of a 50 Hz period) and uses the peak-to-peak delta, averaged over the last 64 readings.
2. When anything is plugged in and drawing current (`A_START`), the hood **light** turns on.
3. When the current is in the kettle's range (`A_MIN`…`A_MAX`), the fan timer starts and the fan steps up:

   | Time after kettle switched on | Fan level |
   |---|---|
   | 1:15 (`BOILING_TIME/4`) | 1 |
   | 2:30 (`BOILING_TIME/2`) | 2 |
   | 3:45 | `BOILING_TIME/4*3`| 3 |
   | 4:50 (`BOILING_TIME - 10 s`) | 4 (boost) |

4. When the kettle switches off, the fan keeps running for another 15 s (`FAN_EXTRA`) and then turns off. The light turns off once the socket current drops below `A_START`.

The IR receiver also hears the Arduino's own transmissions, and the sketch uses that as confirmation. If a command isn't confirmed within 50 ms, it is sent again. After 5 failed attempts the sketch resets everything and sends Fan Off / Light Off.

## Hardware

| Module | Purpose | Arduino UNO pin |
|---|---|---|
| ZMCT103C | Current sensor (kettle power wire passes through the coil) | A0 |
| KY-005 | IR transmitter (to the hood) | D3 |
| KY-022 | IR receiver (confirmations + remote control) | D2 |

All modules use GND and 5 V.

## Remote control (Sony TV remote)

The rarely used teletext buttons of a Sony remote are used for debugging and manual control. Switch off the remote's Bluetooth mode so that it sends infrared.

| Button | Action |
|---|---|
| RED | Cancel everything: fan and light off, leave manual/test mode |
| GREEN | Manual mode, cycling: light on → fan 1 → fan 2 → fan 3 → fan 4 → all off. Automation stays disabled until RED is pressed |
| YELLOW | Test mode on/off. In test mode a Hob2Hood command is sent every 5 seconds |

A Sony remote sends each command several times (not as IR repeats), so the sketch ignores further frames for 100 ms after a button press.

To use another remote, read its hash codes from the Serial Monitor (unrecognised codes are printed) and replace `SONY_RED`, `SONY_GREEN` and `SONY_YELLOW`.

## Configuration

These values were set by experiment for the author's kettle and need to be adjusted for yours:

| Define | Default | Meaning |
|---|---|---|
| `BOILING_TIME` | 5 min | Time for a full kettle to boil; fan levels are derived from it |
| `FAN_EXTRA` | 15 s | Fan run-on after the kettle switches off |
| `A_START` | 50 | Above this, something is plugged in → light on |
| `A_MIN` / `A_MAX` | 550 / 675 | Sensor range treated as "kettle is boiling" |

To calibrate, print `uiAverage` in `vAutomation()` to the Serial Monitor while the kettle is off, standing on its base, and boiling.

## Building

- Board: Arduino UNO (`arduino:avr:uno`)
- Library: [IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote) 4.x (tested with 4.7.1)
- Serial Monitor: 115200 baud


## Limitations

- Kettle detection uses the current level only. Another appliance drawing current in the same range would be treated as a kettle.
- The 15 s fan run-on uses `delay()`, so remote buttons are ignored during that time.

## License

Hobbyists may reuse the code without limitation.
