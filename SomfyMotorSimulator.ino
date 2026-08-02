#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// --- Pin Configurations ---
const byte GDO0_PIN = 19; // CC1101 GDO0 connected to Pin 19 (Hardware Interrupt 4 on Mega)

const byte MOTOR_PIN       = 12; // Simulated Motor 2 Pin (HIGH = Running, LOW = Stopped)
const byte LED_PIN         = 13; // Onboard LED / Simulated Motor 1 Pin
const byte BUTTON_PIN      = 4;  // Manual Pairing Mode Button for Motor 1 (Active Low, Pull-up)

// --- RTS Timings and Tolerances ---
#define SYMBOL 640
#define TOLERANCE_MIN 0.7
#define TOLERANCE_MAX 1.3

const uint32_t tempo_synchro_hw_min  = SYMBOL * 4 * TOLERANCE_MIN;
const uint32_t tempo_synchro_hw_max  = SYMBOL * 4 * TOLERANCE_MAX;
const uint32_t tempo_synchro_sw_min  = 4850 * TOLERANCE_MIN;
const uint32_t tempo_synchro_sw_max  = 4850 * TOLERANCE_MAX;
const uint32_t tempo_half_symbol_min = SYMBOL * TOLERANCE_MIN;
const uint32_t tempo_half_symbol_max = SYMBOL * TOLERANCE_MAX;
const uint32_t tempo_symbol_min      = SYMBOL * 2 * TOLERANCE_MIN;
const uint32_t tempo_symbol_max      = SYMBOL * 2 * TOLERANCE_MAX;

const int16_t bitMin = SYMBOL * TOLERANCE_MIN;

// --- Decoder States and Structures ---
enum t_status {
    waiting_synchro = 0,
    receiving_data  = 1
};

struct somfy_rx_t {
    volatile t_status status;
    volatile uint8_t bit_length;
    volatile uint8_t cpt_synchro_hw;
    volatile uint8_t cpt_bits;
    volatile uint8_t previous_bit;
    volatile bool waiting_half_symbol;
    volatile uint8_t payload[10];

    void clear() {
        status = waiting_synchro;
        bit_length = 56;
        cpt_synchro_hw = 0;
        cpt_bits = 0;
        previous_bit = 0;
        waiting_half_symbol = false;
        memset((void*)payload, 0, sizeof(payload));
    }
};

volatile somfy_rx_t somfy_rx;
volatile bool frame_received = false;
somfy_rx_t ready_frame;

// Diagnostic counter
volatile unsigned long interrupt_count = 0;

// --- Paired Remotes EEPROM Layout ---
// NOTE: EEPROM data is preserved across sketch uploads.
// Only change the EEPROM_MAGIC constant if you modify the EEPROM_Layout struct
// structure in the future. Changing it forces a clean format of the storage.
#define EEPROM_MAGIC 0x5C 
#define MAX_REMOTES  7

struct PairedRemote {
    uint32_t address;
    uint16_t lastRollingCode;
    uint8_t motorId; // 1 = LED (Pin 13), 2 = Pin 12
};

struct EEPROM_Layout {
    uint8_t magic;
    uint8_t numRemotes;
    PairedRemote remotes[MAX_REMOTES];
    float frequency; // Saved radio frequency
};

EEPROM_Layout pairedData;

struct somfy_frame_t {
    bool valid;
    byte cmd;
    uint32_t remoteAddress;
    uint16_t rollingCode;
    byte encKey;
    byte checksum;
};

// --- Pairing & Motor Simulation States ---
byte pairingMotorId = 0; // 0 = None, 1 = Motor 1 (LED), 2 = Motor 2 (Pin 12)
unsigned long pairingModeStartTime = 0;
const unsigned long pairingTimeout = 30000; // 30 seconds pairing window

bool motor1Running = false;
unsigned long motor1StartTime = 0;

bool motor2Running = false;
unsigned long motor2StartTime = 0;

const unsigned long motorRunDuration = 15000; // Auto-stop after 15 seconds

// Mode States
bool rtsSnifferMode = true; // Default to true so users see incoming data immediately
bool debugNoise = false;     // Display invalid checksum packets for low-level debugging

// Global tracking to filter repeats of the same command press
uint32_t lastProcessedAddress     = 0;
uint16_t lastProcessedRollingCode = 0;
unsigned long lastProcessedTime   = 0;

// --- Interrupt Handler for GDO0 (Pin 19) ---
void handleReceive() {
    interrupt_count++;
    
    static unsigned long last_time = 0;
    const unsigned long time = micros();
    const unsigned int duration = time - last_time;

    if (duration < bitMin) {
        return; // Noise filter
    }
    last_time = time;

    switch (somfy_rx.status) {
    case waiting_synchro:
        if (duration > tempo_synchro_hw_min && duration < tempo_synchro_hw_max) {
            somfy_rx.cpt_synchro_hw++;
        }
        else if (duration > tempo_synchro_sw_min && duration < tempo_synchro_sw_max && somfy_rx.cpt_synchro_hw >= 4) {
            memset((void*)somfy_rx.payload, 0, sizeof(somfy_rx.payload));
            somfy_rx.previous_bit = 0;
            somfy_rx.waiting_half_symbol = false;
            somfy_rx.cpt_bits = 0;

            // Determine protocol bit length from hardware sync count
            if (somfy_rx.cpt_synchro_hw <= 7) somfy_rx.bit_length = 56;
            else if (somfy_rx.cpt_synchro_hw == 14) somfy_rx.bit_length = 56;
            else if (somfy_rx.cpt_synchro_hw == 13) somfy_rx.bit_length = 80;
            else if (somfy_rx.cpt_synchro_hw == 12) somfy_rx.bit_length = 80;
            else if (somfy_rx.cpt_synchro_hw > 17) somfy_rx.bit_length = 80;
            else somfy_rx.bit_length = 56;

            somfy_rx.status = receiving_data;
        }
        else {
            somfy_rx.cpt_synchro_hw = 0;
        }
        break;

    case receiving_data:
        if (duration > tempo_symbol_min && duration < tempo_symbol_max && !somfy_rx.waiting_half_symbol) {
            somfy_rx.previous_bit = 1 - somfy_rx.previous_bit;
            somfy_rx.payload[somfy_rx.cpt_bits / 8] |= (somfy_rx.previous_bit << (7 - (somfy_rx.cpt_bits % 8)));
            somfy_rx.cpt_bits++;
        }
        else if (duration > tempo_half_symbol_min && duration < tempo_half_symbol_max) {
            if (somfy_rx.waiting_half_symbol) {
                somfy_rx.waiting_half_symbol = false;
                somfy_rx.payload[somfy_rx.cpt_bits / 8] |= (somfy_rx.previous_bit << (7 - (somfy_rx.cpt_bits % 8)));
                somfy_rx.cpt_bits++;
            }
            else {
                somfy_rx.waiting_half_symbol = true;
            }
        }
        else {
            // Timing error, restart synchro search
            somfy_rx.clear();
        }
        break;
    }

    // Packet fully received
    if (somfy_rx.status == receiving_data && somfy_rx.cpt_bits >= somfy_rx.bit_length) {
        if (!frame_received) {
            memcpy((void*)&ready_frame, (void*)&somfy_rx, sizeof(somfy_rx_t));
            frame_received = true;
        }
        somfy_rx.clear();
    }
}

// --- Frame Decryption & Checksum Validation ---
bool decodeFrame(byte* frame, uint8_t bitLength, somfy_frame_t &f) {
    byte decoded[10];
    decoded[0] = frame[0];
    decoded[7] = frame[7];
    decoded[8] = frame[8];
    decoded[9] = frame[9];
    
    // De-obfuscation (XOR with previous byte)
    for (byte i = 1; i < 7; i++) {
        decoded[i] = frame[i] ^ frame[i - 1];
    }

    // Checksum calculation (ignores lower nibble of byte 1, which holds the checksum)
    byte checksum = 0;
    for (byte i = 0; i < 7; i++) {
        if (i == 1) checksum = checksum ^ (decoded[i] >> 4);
        else checksum = checksum ^ decoded[i] ^ (decoded[i] >> 4);
    }
    checksum &= 0b1111;

    f.checksum = decoded[1] & 0b1111;
    f.encKey = decoded[0];
    f.cmd = decoded[1] >> 4;
    f.rollingCode = decoded[3] + (decoded[2] << 8);
    f.remoteAddress = decoded[6] + ((uint32_t)decoded[5] << 8) + ((uint32_t)decoded[4] << 16);

    // RTS / RTW / RTV Protocol Extensions Resolution
    if (f.cmd == 0xF) { // RTWProto command prefix
        if (f.encKey >= 160) {
            // Standard RTS with Toggle command
            if (f.encKey == 164) f.cmd = 0xC; // Toggle
        }
        else if (f.encKey > 148) {
            // RTV Protocol variant
            f.cmd = f.encKey - 148;
        }
        else if (f.encKey > 133) {
            // RTW Protocol variant
            f.cmd = f.encKey - 133;
        }
    }

    // Validation
    f.valid = (f.checksum == checksum) && (f.remoteAddress > 0) && (f.remoteAddress < 16777215);
    if (f.cmd != 0xE && f.valid) { // 0xE is Sensor
        f.valid = (f.rollingCode > 0);
    }
    if (f.valid && f.encKey == 0) f.valid = false;
    
    return f.valid;
}

// --- EEPROM Helper Functions ---
void loadEEPROM() {
    EEPROM.get(0, pairedData);
    if (pairedData.magic != EEPROM_MAGIC) {
        Serial.println(F("[EEPROM] No valid data found. Initializing storage..."));
        pairedData.magic = EEPROM_MAGIC;
        pairedData.numRemotes = 0;
        pairedData.frequency = 433.92; // Default frequency is 433.92 MHz
        memset(pairedData.remotes, 0, sizeof(pairedData.remotes));
        saveEEPROM();
    } else {
        Serial.print(F("[EEPROM] Loaded. Current Frequency: "));
        Serial.print(pairedData.frequency, 2);
        Serial.println(F(" MHz"));
        Serial.print(F("[EEPROM] Paired remotes count: "));
        Serial.println(pairedData.numRemotes);
        for (int i = 0; i < pairedData.numRemotes; i++) {
            Serial.print(F("  - Remote "));
            Serial.print(i + 1);
            Serial.print(F(": Address 0x"));
            Serial.print(pairedData.remotes[i].address, HEX);
            Serial.print(F(", Last Rolling Code: "));
            Serial.print(pairedData.remotes[i].lastRollingCode);
            Serial.print(F(", Assigned to: Motor "));
            Serial.println(pairedData.remotes[i].motorId);
        }
    }
}

void saveEEPROM() {
    EEPROM.put(0, pairedData);
    Serial.println(F("[EEPROM] Saved successfully."));
}

int findRemoteIndex(uint32_t address) {
    for (int i = 0; i < pairedData.numRemotes; i++) {
        if (pairedData.remotes[i].address == address) {
            return i;
        }
    }
    return -1;
}

bool addRemote(uint32_t address, uint16_t rollingCode, uint8_t motorId) {
    int idx = findRemoteIndex(address);
    if (idx != -1) {
        pairedData.remotes[idx].lastRollingCode = rollingCode;
        pairedData.remotes[idx].motorId = motorId;
        saveEEPROM();
        return true;
    }
    if (pairedData.numRemotes >= MAX_REMOTES) {
        return false;
    }
    pairedData.remotes[pairedData.numRemotes].address = address;
    pairedData.remotes[pairedData.numRemotes].lastRollingCode = rollingCode;
    pairedData.remotes[pairedData.numRemotes].motorId = motorId;
    pairedData.numRemotes++;
    saveEEPROM();
    return true;
}

bool removeRemote(uint32_t address) {
    int idx = findRemoteIndex(address);
    if (idx == -1) return false;

    for (int i = idx; i < pairedData.numRemotes - 1; i++) {
        pairedData.remotes[i] = pairedData.remotes[i + 1];
    }
    pairedData.numRemotes--;
    saveEEPROM();
    return true;
}

// --- Motor Simulation Feedback (Jogs & Relays) ---
void simulateJog(byte motorId) {
    byte pin = (motorId == 1) ? LED_PIN : MOTOR_PIN;
    
    Serial.print(F("\n>>> MOTOR "));
    Serial.print(motorId);
    Serial.println(F(" JOG: UP (Moving brief up) <<<"));
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);
    delay(200);

    Serial.print(F(">>> MOTOR "));
    Serial.print(motorId);
    Serial.println(F(" JOG: DOWN (Moving brief down) <<<"));
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);
    delay(200);
    Serial.print(F(">>> MOTOR "));
    Serial.print(motorId);
    Serial.println(F(" JOG: COMPLETED <<<\n"));
}

void executeMotorCommand(byte cmd, byte motorId) {
    byte pin = (motorId == 1) ? LED_PIN : MOTOR_PIN;
    
    if (cmd == 0x2) { // UP
        Serial.print(F(">>> MOTOR "));
        Serial.print(motorId);
        Serial.println(F(" RUNNING: UP <<<"));
        digitalWrite(pin, HIGH);
        if (motorId == 1) {
            motor1Running = true;
            motor1StartTime = millis();
        } else {
            motor2Running = true;
            motor2StartTime = millis();
        }
    }
    else if (cmd == 0x4) { // DOWN
        Serial.print(F(">>> MOTOR "));
        Serial.print(motorId);
        Serial.println(F(" RUNNING: DOWN <<<"));
        digitalWrite(pin, HIGH);
        if (motorId == 1) {
            motor1Running = true;
            motor1StartTime = millis();
        } else {
            motor2Running = true;
            motor2StartTime = millis();
        }
    }
    else if (cmd == 0x1) { // MY / STOP
        Serial.print(F(">>> MOTOR "));
        Serial.print(motorId);
        Serial.println(F(" STOPPED <<<"));
        digitalWrite(pin, LOW);
        if (motorId == 1) motor1Running = false;
        else motor2Running = false;
    }
}

const char* getCommandName(byte cmd) {
    switch (cmd) {
        case 0x1: return "MY / STOP";
        case 0x2: return "UP";
        case 0x3: return "MY + UP";
        case 0x4: return "DOWN";
        case 0x5: return "MY + DOWN";
        case 0x6: return "UP + DOWN";
        case 0x7: return "MY + UP + DOWN";
        case 0x8: return "PROG (Programming)";
        case 0x9: return "SUN FLAG";
        case 0xA: return "FLAG";
        case 0xB: return "STEP DOWN";
        case 0xC: return "TOGGLE";
        case 0xE: return "SENSOR";
        default:  return "UNKNOWN";
    }
}

// --- RTS Frame Processor (Only processes motor actions for paired remotes) ---
void processFrame(somfy_frame_t &f) {
    // Debounce PROG commands to prevent repeated RF frames from double-triggering state changes
    static uint32_t lastProgAddress = 0;
    static unsigned long lastProgTime = 0;
    if (f.cmd == 0x8) {
        if (f.remoteAddress == lastProgAddress && millis() - lastProgTime < 3000) {
            return; // Ignore repeated frame within 3 seconds
        }
        lastProgAddress = f.remoteAddress;
        lastProgTime = millis();
    }

    int remoteIdx = findRemoteIndex(f.remoteAddress);
    bool isPaired = (remoteIdx != -1);

    if (f.cmd == 0x8) { // PROG
        if (pairingMotorId > 0) {
            if (isPaired) {
                // Already paired remote sends PROG during pairing mode -> Unpair
                removeRemote(f.remoteAddress);
                Serial.print(F(">>> Unpaired/Deleted Remote Address: 0x"));
                Serial.println(f.remoteAddress, HEX);
                simulateJog(pairedData.remotes[remoteIdx].motorId);
            } else {
                // New remote sends PROG during pairing mode -> Pair with active motor
                if (addRemote(f.remoteAddress, f.rollingCode, pairingMotorId)) {
                    Serial.print(F(">>> Paired/Saved New Remote Address: 0x"));
                    Serial.print(f.remoteAddress, HEX);
                    Serial.print(F(" to Motor "));
                    Serial.println(pairingMotorId);
                    simulateJog(pairingMotorId);
                } else {
                    Serial.println(F("[ERROR] Pairing failed. Paired remotes storage is full!"));
                }
            }
            pairingMotorId = 0;
            digitalWrite(LED_PIN, LOW);
            digitalWrite(MOTOR_PIN, LOW);
        } else {
            // PROG sent outside of active pairing mode
            if (isPaired) {
                // Existing remote requests its assigned motor to enter pairing mode
                byte assignedMotor = pairedData.remotes[remoteIdx].motorId;
                pairingMotorId = assignedMotor;
                pairingModeStartTime = millis();
                Serial.print(F(">>> PROG on paired remote: Entering pairing mode for Motor "));
                Serial.print(assignedMotor);
                Serial.println(F(" (30s)..."));
                simulateJog(assignedMotor);
            } else {
                Serial.println(F(">>> PROG on unpaired remote ignored (system not in pairing mode)."));
            }
        }
    } else {
        // Normal operation commands (UP, DOWN, MY)
        if (isPaired) {
            uint16_t lastCode = pairedData.remotes[remoteIdx].lastRollingCode;
            byte assignedMotor = pairedData.remotes[remoteIdx].motorId;

            // Debounce standard repeat frames of the exact same button press
            if (f.remoteAddress == lastProcessedAddress && 
                f.rollingCode == lastProcessedRollingCode && 
                millis() - lastProcessedTime < 1500) {
                return; // Ignore repeat packet of the same press
            }

            lastProcessedAddress     = f.remoteAddress;
            lastProcessedRollingCode = f.rollingCode;
            lastProcessedTime        = millis();

            if (f.rollingCode < lastCode) {
                Serial.print(F("[WARN] Rolling code went backwards (stored: "));
                Serial.print(lastCode);
                Serial.print(F(", Recv: "));
                Serial.print(f.rollingCode);
                Serial.println(F("). Replay or remote reset warning, executing anyway."));
            }

            // Only update EEPROM if the rolling code value is actually different
            if (f.rollingCode != lastCode) {
                pairedData.remotes[remoteIdx].lastRollingCode = f.rollingCode;
                saveEEPROM();
            }

            // Execute action on assigned motor
            executeMotorCommand(f.cmd, assignedMotor);
        } else {
            // Log that it's ignored for motor actuation (since we already logged the sniffer details)
            if (!rtsSnifferMode) {
                Serial.print(F(">>> Ignored command from unpaired remote: 0x"));
                Serial.println(f.remoteAddress, HEX);
            }
        }
    }
}

void togglePairingMode(byte motorId) {
    if (pairingMotorId == motorId) {
        // Deactivate
        pairingMotorId = 0;
        digitalWrite(LED_PIN, LOW);
        digitalWrite(MOTOR_PIN, LOW);
        Serial.print(F("\n[SERIAL] Exiting pairing mode for Motor "));
        Serial.println(motorId);
    } else {
        // Activate
        pairingMotorId = motorId;
        pairingModeStartTime = millis();
        Serial.print(F("\n[SERIAL] Entering pairing mode for Motor "));
        Serial.print(motorId);
        Serial.println(F(" (30s window)..."));
        simulateJog(motorId);
    }
}

// --- Arduino Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("\n=============================================="));
    Serial.println(F("   Somfy RTS Motor/Receiver Simulator v1.9   "));
    Serial.println(F("=============================================="));

    // Configure Pin Modes
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(GDO0_PIN, INPUT);

    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    // Load paired remotes and frequency from non-volatile storage
    loadEEPROM();

    // 1. Initialize CC1101 with Mega Hardware defaults
    Serial.println(F("[CC1101] Initializing transceiver..."));
    ELECHOUSE_cc1101.Init();
    
    // 2. Load configuration options matching the saved frequency in EEPROM
    ELECHOUSE_cc1101.setMHZ(pairedData.frequency); // Dynamic frequency selection
    ELECHOUSE_cc1101.setModulation(2);             // ASK/OOK Modulation
    ELECHOUSE_cc1101.setRxBW(812.50);               // Maximum Bandwidth (812.50 kHz)
    ELECHOUSE_cc1101.SetRx();                       // Alıcı Moduna Al

    if (ELECHOUSE_cc1101.getCC1101()) {
        Serial.print(F("[CC1101] Transceiver active and listening on: "));
        Serial.print(pairedData.frequency, 2);
        Serial.println(F(" MHz"));
    } else {
        Serial.println(F("[CC1101] Connection failed! Verify SPI & VCC/GND wiring."));
    }

    // Attach Hardware Interrupt on Pin 19 (GDO0)
    somfy_rx.clear();
    attachInterrupt(digitalPinToInterrupt(GDO0_PIN), handleReceive, CHANGE);
    Serial.println(F("[INTERRUPT] Manchester decoder ISR attached to Pin 19."));

    // Initial startup pairing window (20 seconds) - Assigned to Motor 1 (LED) by default
    pairingMotorId = 1;
    pairingModeStartTime = millis();
    Serial.println(F("[PAIRED] Entering startup pairing window for Motor 1 (LED) for 20 seconds. Send PROG to pair."));
    
    Serial.println(F("\nCommands available in Serial Monitor:"));
    Serial.println(F("  'l'      - Toggle Live RTS Sniffer Mode (shows all codes like ESPSomfy-RTS)"));
    Serial.println(F("  'v'      - View/List currently paired remotes in EEPROM"));
    Serial.println(F("  'f[mhz]' - Change frequency dynamically and save to EEPROM (e.g. f433.42 or f433.92)"));
    Serial.println(F("  'p1'     - Toggle manual pairing mode for Motor 1 (LED Pin 13)"));
    Serial.println(F("  'p2'     - Toggle manual pairing mode for Motor 2 (Pin 12)"));
    Serial.println(F("  'c'      - Clear all paired remotes from EEPROM (Factory Reset)"));
    Serial.println(F("  'd'      - Toggle noise debug print (shows packets failing checksum)"));
    Serial.println(F("=============================================="));
}

// --- Main Program Loop ---
void loop() {
    // 1. Process Received RTS RF Frames (Thread-Safe Handoff)
    bool has_frame = false;
    somfy_rx_t local_frame;

    noInterrupts();
    if (frame_received) {
        has_frame = true;
        memcpy((void*)&local_frame, (void*)&ready_frame, sizeof(somfy_rx_t));
        frame_received = false;
    }
    interrupts();

    if (has_frame) {
        somfy_frame_t f;
        bool isValid = decodeFrame((byte*)local_frame.payload, local_frame.bit_length, f);
        
        // --- Live RTS Sniffer Output ---
        if (rtsSnifferMode) {
            if (isValid) {
                Serial.print(F("[RTS SNIFFER] Key: 0x"));
                if (f.encKey < 0x10) Serial.print('0');
                Serial.print(f.encKey, HEX);
                Serial.print(F(" | Cmd: "));
                Serial.print(getCommandName(f.cmd));
                Serial.print(F(" (0x"));
                Serial.print(f.cmd, HEX);
                Serial.print(F(") | Addr: 0x"));
                Serial.print(f.remoteAddress, HEX);
                Serial.print(F(" | Rcode: "));
                Serial.print(f.rollingCode);
                Serial.print(F(" | Checksum: 0x"));
                Serial.print(f.checksum, HEX);
                Serial.print(F(" (VALID) | Raw: "));
                for (int i = 0; i < 7; i++) {
                    if (local_frame.payload[i] < 0x10) Serial.print('0');
                    Serial.print(local_frame.payload[i], HEX);
                    Serial.print(' ');
                }
                Serial.println();
            } else if (debugNoise) {
                Serial.print(F("[RTS SNIFFER] Key: 0x"));
                if (f.encKey < 0x10) Serial.print('0');
                Serial.print(f.encKey, HEX);
                Serial.print(F(" | Cmd: 0x"));
                Serial.print(f.cmd, HEX);
                Serial.print(F(" | Addr: 0x"));
                Serial.print(f.remoteAddress, HEX);
                Serial.print(F(" | Rcode: "));
                Serial.print(f.rollingCode);
                Serial.print(F(" | Checksum: 0x"));
                Serial.print(f.checksum, HEX);
                Serial.print(F(" (INVALID) | Raw: "));
                for (int i = 0; i < 7; i++) {
                    if (local_frame.payload[i] < 0x10) Serial.print('0');
                    Serial.print(local_frame.payload[i], HEX);
                    Serial.print(' ');
                }
                Serial.println();
            }
        }

        if (isValid) {
            processFrame(f);
        }
    }

    // 2. Handle Manual Button Press for Pairing Mode (Motor 1 LED by default)
    static bool lastBtnState = HIGH;
    bool btnState = digitalRead(BUTTON_PIN);
    if (btnState == LOW && lastBtnState == HIGH) {
        delay(50); // Simple Debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
            togglePairingMode(1);
        }
    }
    lastBtnState = btnState;

    // 3. Handle Serial Commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        if (cmd == 'l' || cmd == 'L') {
            rtsSnifferMode = !rtsSnifferMode;
            Serial.print(F("\n[SERIAL] Live RTS Sniffer Mode: "));
            Serial.println(rtsSnifferMode ? F("ENABLED (Printing all valid RTS frames)") : F("DISABLED"));
        }
        else if (cmd == 'v' || cmd == 'V') {
            Serial.println(F("\n[SERIAL] Listing currently paired remotes:"));
            loadEEPROM();
        }
        else if (cmd == 'f' || cmd == 'F') {
            delay(50); // Wait briefly for digits to arrive in buffer
            float freq = Serial.parseFloat();
            
            if (freq >= 300.0 && freq <= 928.0) {
                pairedData.frequency = freq;
                saveEEPROM();
                
                // Set the frequency dynamically on the CC1101
                ELECHOUSE_cc1101.setMHZ(freq);
                ELECHOUSE_cc1101.SetRx();
                
                Serial.print(F("\n[CC1101] Frequency changed dynamically to: "));
                Serial.print(freq, 2);
                Serial.println(F(" MHz"));
            } else {
                Serial.println(F("\n[ERROR] Invalid frequency! Must be between 300.0 and 928.0 MHz."));
            }
        }
        else if (cmd == 'p' || cmd == 'P') {
            delay(15); // Wait briefly to check if '1' or '2' follows
            char next = '1';
            if (Serial.available() > 0) {
                next = Serial.read();
            }
            
            byte motorId = (next == '2') ? 2 : 1;
            togglePairingMode(motorId);
        }
        else if (cmd == 'c' || cmd == 'C') {
            pairedData.numRemotes = 0;
            saveEEPROM();
            Serial.println(F("\n[SERIAL] Cleared all paired remotes from storage."));
            digitalWrite(MOTOR_PIN, LOW);
            digitalWrite(LED_PIN, LOW);
            motor1Running = false;
            motor2Running = false;
        }
        else if (cmd == 'd' || cmd == 'D') {
            debugNoise = !debugNoise;
            Serial.print(F("\n[SERIAL] Noise debugging output: "));
            Serial.println(debugNoise ? F("ENABLED") : F("DISABLED"));
        }
        
        // Clear out any trailing newlines
        while (Serial.peek() == '\n' || Serial.peek() == '\r') {
            Serial.read();
        }
    }

    // 4. Handle Pairing Mode Timeouts & Visual Indicators (LED Blinking)
    if (pairingMotorId > 0) {
        unsigned long elapsed = millis() - pairingModeStartTime;
        unsigned long timeout = (pairingModeStartTime < 5000) ? 20000 : pairingTimeout;
        
        if (elapsed > timeout) {
            digitalWrite(LED_PIN, LOW);
            digitalWrite(MOTOR_PIN, LOW);
            Serial.print(F("\n[TIMEOUT] Pairing window expired for Motor "));
            Serial.print(pairingMotorId);
            Serial.println(F(". Returning to normal listening mode."));
            pairingMotorId = 0;
        } else {
            // Rapid blink the corresponding motor pin to show pairing mode is active (every 100ms)
            byte blinkPin = (pairingMotorId == 1) ? LED_PIN : MOTOR_PIN;
            static unsigned long lastBlink = 0;
            if (millis() - lastBlink > 100) {
                digitalWrite(blinkPin, !digitalRead(blinkPin));
                lastBlink = millis();
            }
        }
    }

    // 5. Handle Motor Run Timeout (Auto-Stop simulation)
    if (motor1Running && (millis() - motor1StartTime > motorRunDuration)) {
        digitalWrite(LED_PIN, LOW);
        motor1Running = false;
        Serial.println(F("\n>>> [MOTOR 1] Auto-stop: Limit reached (15 seconds elapsed). <<<"));
    }
    if (motor2Running && (millis() - motor2StartTime > motorRunDuration)) {
        digitalWrite(MOTOR_PIN, LOW);
        motor2Running = false;
        Serial.println(F("\n>>> [MOTOR 2] Auto-stop: Limit reached (15 seconds elapsed). <<<"));
    }
}
