# FD-200 Remote: ESP32 Web Controller for the ISDT FD-200 Discharger

Control an **ISDT FD-200 battery discharger** from any web browser, using an ESP32 as a Bluetooth bridge.

Some phone and app combinations can no longer connect to the FD-200. For example, a Pixel 7 running the current ISDT GO app couldn't connect. This project skips the app entirely. The ESP32 talks to the discharger over Bluetooth Low Energy (BLE) and serves a simple web page. You can use that page from your phone, tablet or PC to start and stop discharges.

<!-- Add a screenshot of the web UI here, for example:
![Web UI](docs/screenshot.png)
-->

## Features

- **Start and stop discharging** from a web page, with no app needed.
- **Set the cell count (1–8S), the cutoff voltage per cell, and the discharge current** (5 / 10 / 15 / 20 / 25 A).
- **Named presets.** Save each pack or drone with its own cell count, current and cutoff, then pick it from a list. The Start button always shows exactly what it's about to do, for example "Start: Five33 6S – 6S at 10 A". If you edit a field after picking a preset, the page warns you that you're no longer using it.
- **Scan for and pick your FD-200 from the web page.** No Bluetooth address is hardcoded. Your choice is saved and it reconnects automatically after a power cycle.
- **Home WiFi support with automatic hotspot fallback.** Join your home network from the web page. If home WiFi can't be found, the ESP32 starts its own hotspot so you can always reach it.
- **Live readout** of the run state, pack voltage, cell voltages, the device's active settings and the run time. See [Known limitations](#known-limitations).
- Everything is stored on the ESP32 (saved discharger, WiFi, presets), so it works the same from any phone or computer.

## What you need

### Hardware

| Item | Notes |
|---|---|
| **ESP32 development board** | Must have **both WiFi and Bluetooth LE**. The recommended board, and the one this was built on, is a classic **ESP32 (ESP32-WROOM-32) dev kit**, such as the ESP32-DevKitC, DOIT ESP32 DevKit V1 or similar 30/38-pin boards. A 4 MB flash board is fine. |
| USB cable | It must be a **data** cable. Many cheap cables are charge-only. |
| ISDT FD-200 | The discharger itself, with a battery connected as usual. |
| 5 V USB power | For permanent use, any USB phone charger can power the ESP32. |

**Which ESP32 chips work:**

| Chip | Works? |
|---|---|
| ESP32 (original, WROOM-32 / WROVER) | ✅ Yes. This is the one it was built and tested on. |
| ESP32-S3, ESP32-C3, ESP32-C6 | ⚠️ Should work (they have WiFi + BLE), but untested. |
| ESP32-S2 | ❌ No. It has no Bluetooth. |
| ESP8266 | ❌ No. It has no Bluetooth. |

### Software

- [Arduino IDE](https://www.arduino.cc/en/software) (2.x recommended)
- The **esp32 board package by Espressif Systems**, installed through the Boards Manager (see below)
- No extra libraries are needed. WiFi, WebServer, Preferences, ESPmDNS and BLE all come with the ESP32 board package.

## Installation

1. **Install the ESP32 board package**
   - In the Arduino IDE, open **File → Preferences** and add this to *Additional boards manager URLs*:
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```
   - Open **Tools → Board → Boards Manager**, search for **esp32**, and install **esp32 by Espressif Systems**.

2. **Get the code**
   - Download or clone this repository.
   - The Arduino IDE needs the sketch file to sit in a folder with the same name. For example: `fd200_controller/fd200_controller.ino`.

3. **(Optional) Change the hotspot name and password.** Near the top of the sketch:
   ```cpp
   #define AP_SSID "FD200-Discharger"
   #define AP_PASS "12345678"   // 8+ characters. Change this!
   #define HOSTNAME "fd200"     // http://fd200.local on your home network
   ```

4. **Select the board and settings**
   - **Tools → Board → esp32 → ESP32 Dev Module** (or the entry that matches your board)
   - **Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)**

     WiFi and Bluetooth together make a large program. If you see **"Sketch too big"**, this setting fixes it.
   - **Tools → Port →** the port your ESP32 is on

5. **Upload.** If the upload stalls at "Connecting…", hold the **BOOT** button on the ESP32 until it starts writing.

6. **(Optional) Watch the Serial Monitor at 115200 baud.** It prints the web address, connection status and every Bluetooth frame sent and received. This is handy for troubleshooting.

## First-time setup

1. **Close the ISDT app on your phone, or turn off its Bluetooth.** The FD-200 only accepts one Bluetooth connection at a time, and it stops advertising while the phone is connected.
2. **Power on the FD-200 and the ESP32.**
3. **On your phone or PC, join the WiFi network `FD200-Discharger`** (default password `12345678`).
4. **Open `http://192.168.4.1` in a browser.**
5. **Under Discharger, tap Scan for FD-200**, then tap your FD-200 in the list. It shows as connected within a few seconds. If it doesn't appear, tick *Show all Bluetooth devices*.
6. **(Optional) Under Home WiFi:**
   - Tap **Scan WiFi networks** and pick your network.
   - Enter the password and tap **Save & connect**.
   - The page shows the ESP32's new address once it joins. From then on, open that address (or `http://fd200.local`) while on your home WiFi.

## Using it

1. **Pick a preset, or set the values by hand:**
   - **Cells:** the battery's cell count (1–8S).
   - **Cutoff per cell:** where the discharge stops. For example, 3.70–3.85 V for LiPo storage.
   - **Discharge current:** 5, 10, 15, 20 or 25 A.
2. **Check that the Start button shows the cell count and current you expect, then tap it.**
3. **Watch the Live section.** *Device setting* shows what the FD-200 itself reports it is running.
4. **Tap Stop Discharge to end early.** Otherwise the FD-200 stops by itself at the cutoff.

**Presets:**
- **To make one:** set the fields, tap **Save as preset** and give it a name.
- **To update one:** save again with the same name.
- **To delete one:** pick it and tap **Delete preset**.

### How the WiFi behaves

| Situation | What happens |
|---|---|
| No home WiFi saved, or it's turned off | Hotspot only: join `FD200-Discharger` and open `http://192.168.4.1` |
| Home WiFi saved | It joins at power-on. If it can't join within ~20 s, the hotspot starts. |
| Home WiFi drops | After 30 s the hotspot starts. It retries home WiFi every 5 minutes, but only while nobody is on the hotspot. |
| Joined home WiFi | The hotspot turns itself off after 60 s with nobody connected to it. |

The **Use home WiFi** checkbox turns home WiFi off or on without forgetting the password. **Clear WiFi settings** erases it completely.

> The ESP32 only supports **2.4 GHz** WiFi. 5 GHz-only networks won't show up.

## Troubleshooting

| Problem | Try |
|---|---|
| FD-200 doesn't show up in the scan | Close the ISDT app and turn off Bluetooth on the phone that used to connect to it. Power-cycle the FD-200. Tick *Show all Bluetooth devices*. Move the ESP32 closer. |
| "Connected, but the FD-200 did not answer the auth frame" or "rejected auth" | See [The auth frame](#the-auth-frame) below. |
| "Sketch too big" when compiling | Set **Tools → Partition Scheme → Huge APP**. |
| Upload stuck on "Connecting…" | Hold the BOOT button during upload, and try a different USB cable (some are charge-only). |
| Can't reach `http://fd200.local` | Some devices, especially Android, don't support `.local` names. Use the IP address shown on the page or in the Serial Monitor. |
| Lost the page after changing WiFi settings | Join the `FD200-Discharger` hotspot and open `http://192.168.4.1`. |
| Device reports a different current than you selected | Please open an issue with the Serial Monitor log (see [Known limitations](#known-limitations)). |

## How it works

```
 Phone / PC browser  <--- WiFi (HTTP) --->  ESP32  <--- Bluetooth LE --->  ISDT FD-200
```

- **Web server:** the ESP32 serves a single-page web UI and a small JSON API.
- **Bluetooth task:** a separate background task (pinned to core 1) handles the BLE link. It connects, authenticates, polls the FD-200 about once a second, and runs commands queued from the web page.
- **Settings storage:** the chosen discharger, the WiFi credentials and the presets are kept in the ESP32's flash (NVS / `Preferences`), so they survive power cycles and re-uploads.

### Protocol notes

The FD-200 protocol was worked out from a Bluetooth capture of the official app talking to the discharger. Everything below is reverse-engineered, so treat it as best-known rather than official.

- **Service / characteristic:** `0000fff0-...` / `0000fff7-0000-1000-8000-00805f9b34fb`. Writes and notifications both go through `fff7`.
- **Frame format:**
  ```
  [len] [0xAA] [dir] [plen] [cmd] [data...] [chk]
  ```
  - `len` = `plen + 4`
  - `dir` = `0x12` from app to device, `0x21` from device to app
  - `plen` = 1 + the number of data bytes
  - `chk` = `(dir + plen + cmd + sum(data)) & 0xFF`
- **Commands:**

  | Request → Reply | Meaning |
  |---|---|
  | `0x18 → 0x19` | Auth / bind. Reply status `00` = OK. |
  | `0xE0 → 0xE1` | Device info (contains `FD200`) |
  | `0xE4 → 0xE5` | Pack voltage + 8 cell voltages (mV, little-endian) |
  | `0xE6 → 0xE7` | Run state (`00` idle, `02` discharging), elapsed time, active cells / cutoff / current |
  | `0xE8 → 0xE9` | Miscellaneous status |
  | `0xEA → 0xEB` | Start (sub-command `02`) / stop (sub-command `03`) |
  | `0x48 → 0x49` | Sent by the app right after start. Purpose unknown; mirrored here. |

- **Start payload (`0xEA`):**
  ```
  00 02 01 01 [current mA lo] [current mA hi] 00 00 [cells] [cutoff mV lo] [cutoff mV hi]
  ```
  For example, 10 A = `10 27` (0x2710 = 10000 mA), and 3.70 V cutoff = `74 0E` (0x0E74 = 3700 mV).
- **Stop payload (`0xEA`):** `00 03 00 01 00 00 00 00 00 00 00`

### HTTP API

You can control it from scripts, Home Assistant, etc. The two `GET` request types use query strings. The `POST` requests take form-encoded fields.

| Method | Endpoint | Purpose |
|---|---|---|
| GET | `/status` | JSON: connection state, live readings, WiFi status |
| GET | `/start?cells=6&volt=3.70&amps=10000` | Start a discharge. `amps` is in mA: 5000 / 10000 / 15000 / 20000 / 25000. |
| GET | `/stop` | Stop the discharge |
| GET | `/scan` | Scan for BLE devices (about 5 s) |
| GET | `/select?addr=..&type=..&name=..` | Choose and save a discharger |
| GET | `/forget` | Forget the saved discharger |
| GET | `/presets` | List presets |
| POST | `/presetsave` (`name, cells, volt, amps`) | Create or update a preset |
| POST | `/presetdel` (`name`) | Delete a preset |
| GET | `/wifiscan` | List nearby WiFi networks |
| POST | `/wifisave` (`ssid, pass`) | Save and join home WiFi |
| POST | `/wifienable` (`on=1/0`) | Turn home WiFi on or off |
| POST | `/wififorget` | Clear the WiFi settings |

> There is no login on the web UI. Anyone who can reach the ESP32 on your network can start or stop a discharge. Keep it on a trusted network, and change the hotspot password.

## The auth frame

When it connects, the FD-200 expects a 22-byte "bind" frame (command `0x18`). The one in the sketch (`AUTH_FRAME`) was captured from a working phone and app. **It is not yet known whether this frame works for every FD-200, or whether it's tied to the phone or app install it came from.** It has worked on the author's unit.

If your FD-200 rejects it, you can capture your own frame:

1. On an Android phone that *can* connect with the ISDT app, enable **Developer options → Enable Bluetooth HCI snoop log**.
2. Toggle Bluetooth off and on, then connect to the FD-200 in the ISDT app.
3. Pull a bug report (`adb bugreport`) and open the `btsnoop_hci.log` inside it with [Wireshark](https://www.wireshark.org/).
4. Filter for writes to the device and find the 22-byte frame starting with `15 AA 12 11 18`.
5. Replace the bytes of `AUTH_FRAME` in the sketch with yours and re-upload.

If you try this on another FD-200, please open an issue saying whether the stock frame worked. That will settle it for everyone.

## Known limitations

- **Current encoding:** the current value in the start command is decoded from a 10 A capture. The 5 / 15 / 20 / 25 A settings follow the same pattern but should be checked. After starting, the *Device setting* line in the Live section shows what the FD-200 reports.
- **Live readout:** the field positions for the live readings are partly decoded. Some values may show as zero or "unknown" depending on device state and firmware.
- **Firmware:** this has only been tried on one FD-200 and its firmware version. Other firmware may behave differently.
- **One connection:** the FD-200 accepts one Bluetooth connection at a time. While the ESP32 is connected, the phone app can't connect, and the other way round.

## Safety and disclaimer

**Discharging lithium batteries at high current produces a lot of heat and carries a real fire risk.** Never leave a discharge unattended. Use a fire-safe surface or bag, and set the cell count and cutoff correctly for your pack. Double-check the cell count and current before you press Start.

This is an independent hobby project. It is **not affiliated with, endorsed by, or supported by ISDT**. It uses a reverse-engineered protocol and is provided as-is, with no warranty. Use it at your own risk.

## Contributing

Issues and pull requests are welcome, especially:
- Reports from other FD-200 units or firmware versions, including whether the stock auth frame worked
- Captures that clarify the unknown protocol fields
- Testing on ESP32-S3 / C3 / C6 boards

When reporting a problem, please include the Serial Monitor output (115200 baud). The `>>` lines are frames sent to the FD-200, and the `<<` lines are its replies.

## License

<!-- Pick a license (e.g. MIT) and add a LICENSE file to the repo. -->
