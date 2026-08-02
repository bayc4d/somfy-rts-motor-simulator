# Somfy RTS Motor & Receiver Simulator (Arduino Mega 2560 + CC1101)

This project is a standalone **Somfy RTS (Radio Technology Somfy) Motor and Receiver Simulator** designed for Arduino Mega 2560 and the CC1101 transceiver module. 

It allows you to simulate up to **two independent physical motors/shades** using simple outputs (the onboard LED and Pin 12). It is highly useful for testing physical Somfy remotes (like Telis, Smoove) or virtual smart-home transceivers (like **ESPSomfy-RTS**) without having physical motors nearby.

---

## Features

*   **Multi-Protocol Decoding**: Decodes standard **RTS**, **RTW**, and **RTV** Somfy protocol variants (including variant command layouts where button codes are embedded inside the encryption key).
*   **Dual Motor Simulation**: Simulates two independent motor channels:
    *   **Motor 1**: Onboard LED (Pin 13).
    *   **Motor 2**: Digital GPIO (Pin 12).
*   **Realistic Pairing Flow**: Replicates genuine Somfy RTS pairing state machines:
    *   Add new remotes, pair virtual transceivers (ESPSomfy-RTS), or delete existing remotes using the remote's **PROG** button.
    *   Simulates motor **JOG** movements (UP/DOWN toggling) to confirm pairing operations.
*   **Smart Debounce Filters**: Implements software time-based debouncing (1.5s for movements, 3s for PROG) to handle redundant RF repeat packages, prevent duplicate triggers, and avoid unnecessary EEPROM writes.
*   **EEPROM Storage**: Saves up to 7 paired remote addresses and their last rolling codes in non-volatile EEPROM (reset-safe).
*   **Live RTS Sniffer Mode**: A built-in terminal sniffer (toggleable via `L`) that prints raw received RTS frame parameters (Key, Cmd, Address, Rolling Code, Checksum, and Raw bytes) in a single clean line—just like the ESPSomfy-RTS logs.
*   **Motor Travel Timeout**: Automatically cuts motor pin outputs after **15 seconds** of continuous operation to protect dummy hardware/relays.

---

## Hardware Configuration (Arduino Mega to CC1101)

> [!WARNING]  
> The CC1101 is a **3.3V device**. Connecting VCC directly to 5V will damage the RF chip. Always power it from the 3.3V pin on the Arduino Mega.

| CC1101 Pin | Arduino Mega Pin | Description |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | Power (Do NOT connect to 5V!) |
| **GND** | **GND** | Common Ground |
| **CSN** | **Pin 53** | SPI Chip Select (Hardware SS) |
| **SCK** | **Pin 52** | SPI Clock |
| **MOSI** | **Pin 51** | SPI MOSI |
| **MISO** | **Pin 50** | SPI MISO |
| **GDO0** | **Pin 19** | Receive Pin (Hardware Interrupt 4, CHANGE mode) |

### Outputs & Inputs:
*   **Motor 1 Output**: Onboard LED (**Pin 13**)
*   **Motor 2 Output**: Digital LED/Relay (**Pin 12**)
*   **Manual Pairing Button**: Tactile button connected between **Pin 4** and **GND** (toggles Motor 1 pairing mode).

---

## Library Requirements

This project relies on the **SmartRC-CC1101-Driver-Lib** (by LSatan) for hardware register operations. 
1. Open the Arduino IDE Library Manager (**Sketch -> Include Library -> Manage Libraries...**).
2. Search for and install **SmartRC-CC1101-Driver-Lib**.

---

## How to Use

### 1. Upload & Connect
1. Upload the code to your Arduino Mega.
2. Open the **Serial Monitor** at **115200** baud rate.
3. Upon startup, the EEPROM will initialize, and **Motor 1** will enter a 20-second startup pairing window (LED Pin 13 blinks rapidly).

### 2. Pairing a Remote Control
To pair a remote control with a simulated motor:
1. Trigger pairing mode for your desired motor:
    *   Type **`p1`** (or **`p`**) in the Serial Monitor for **Motor 1** (LED blinks).
    *   Type **`p2`** in the Serial Monitor for **Motor 2** (Pin 12 output blinks).
2. Press the **PROG** button on your physical remote control.
3. The simulator will detect the address, pair it, save it to the EEPROM, and perform a brief **JOG** simulation.

### 3. Pairing a Secondary Remote or ESPSomfy-RTS
Once a remote (Remote A) is paired, you can link a secondary remote or ESPSomfy-RTS virtual device (Remote B) using the standard Somfy sequence:
1. Press the **PROG** button on **Remote A** (already paired). The associated motor simulator will JOG and enter pairing mode (blinking).
2. Press the **PROG** button on **Remote B** (new remote or ESPSomfy-RTS virtual button).
3. The simulator will pair **Remote B** with the same motor and JOG to confirm.

---

## Serial Terminal Commands

Type the following commands in the Serial Monitor:
*   **`l` / `L`**: Toggle **Live RTS Sniffer Mode** (shows all incoming signals).
*   **`v` / `V`**: View currently paired remotes in the EEPROM and their associated motor channels.
*   **`f[mhz]`**: Change frequency dynamically and save to EEPROM (e.g. f433.42 or f433.92).
*   **`p1`** (or **`p`**): Toggle manual pairing mode for **Motor 1** (LED).
*   **`p2`**: Toggle manual pairing mode for **Motor 2** (Pin 12).
*   **`c` / `C`**: **Factory Reset** (clears all paired remotes from EEPROM).
*   **`d` / `D`**: Toggle noise debug output (prints packets failing checksum).

---

## EEPROM & Code Upload Persistence

By default, uploading a new sketch to the Arduino Mega via the Arduino IDE **does NOT erase the EEPROM memory**. 
*   All paired remotes and your configured frequency will **persist** across code updates.
*   **The Magic Byte**: The code uses `#define EEPROM_MAGIC 0x5C` to detect if the data layout in the EEPROM matches the code. If you make custom changes to the `EEPROM_Layout` struct in the future, change this magic number to force the simulator to cleanly format the EEPROM upon the next boot. Otherwise, leave it as is to keep your paired remotes.

---

## Credits & Acknowledgments

This project is built upon and inspired by the following open-source resources:
*   **[ESPSomfy-RTS](https://github.com/rstrouse/ESPSomfy-RTS)** (by rstrouse) - Provided the framework for Somfy RTS timing parameters, protocol de-obfuscation logic, and variant mapping (RTV/RTW).
*   **[SmartRC-CC1101-Driver-Lib](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib)** (by LSatan) - The CC1101 register interface library used to establish physical OOK/ASK transceiving.
*   **[Somfy RTS Receiver Hackster Project](https://www.hackster.io/frankbeen/somfy-rts-receiver-295382)** (by Frank Been) - A valuable reference explaining hardware-level RTS decoding timing and pulse structures on Arduino.
