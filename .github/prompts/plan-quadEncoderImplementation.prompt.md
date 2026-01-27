# Plan: QuadEncoder TypeScript Implementation

Build a layered micro:bit MakeCode extension for the Yahboom quad encoder motor controller, separating buffer construction, I2C transport, and response parsing for testability and clarity.

## Reference Documentation & Code

**CRITICAL**: All implementation must strictly follow these two authoritative sources. Cross-check between them to catch discrepancies:

1. **Python Reference Implementation**: `docs/IIC.py`
   - Complete working I2C protocol implementation in Python
   - Shows exact byte-level buffer construction and parsing
   - Demonstrates configuration sequences with proper delays
   - Motor parameter tables for all 5 motor types
   - **Key patterns to replicate**:
     - Big-endian encoding: `[(data >> 8) & 0xFF, data & 0xFF]`
     - Sign extension: `if value & 0x8000: value -= 0x10000`
     - 32-bit assembly: `(high_val << 16) | low_val`
     - Float encoding: `struct.pack('<f', value)` (little-endian)
     - EEPROM writes need 0.1s delays: `time.sleep(0.1)`

2. **Hardware Documentation**: https://www.yahboom.net/public/upload/upload-html/1740571339/Drive%20motor%20and%20read%20encoder-USART.html
   - Official Yahboom motor controller specifications
   - Register definitions and communication protocol
   - Hardware capabilities and limitations

**Cross-Validation Strategy**:
- When implementing each function, reference the corresponding Python code section
- Verify register addresses match between Python constants and documentation
- Compare data encoding methods (endianness, sign handling) across both sources
- If discrepancies exist, prioritize the Python reference (it's proven working code)
- Document any ambiguities or differences found during implementation



## Architecture Layers

**Layer 1: Buffer Construction (Pure functions)**
- `buildMotorTypeBuffer(type: MotorType): Buffer` — Create 1-byte motor type payload
- `buildDeadZoneBuffer(value: number): Buffer` — Create 2-byte big-endian deadzone
- `buildReductionRatioBuffer(ratio: number): Buffer` — Create 2-byte phase buffer
- `buildPulsesPerLineBuffer(pulses: number): Buffer` — Create 2-byte line buffer
- `buildWheelDiameterBuffer(diameter: number): Buffer` — Create 4-byte float (little-endian)
- `buildSpeedControlBuffer(m1, m2, m3, m4: number): Buffer` — Create 8-byte speed array
- `buildPWMControlBuffer(m1, m2, m3, m4: number): Buffer` — Create 8-byte PWM array

**Layer 2: I2C Transport (Generic I/O)**
- `writeI2C(register: number, data: Buffer): void` — Write buffer to register at 0x26
- `readI2C(register: number, length: number): Buffer` — Read bytes from register at 0x26

**Layer 3: Response Parsers (Pure functions)**
- `parseEncoder16(buffer: Buffer): number` — Parse 2-byte signed big-endian encoder value
- `parseEncoder32(highBuffer: Buffer, lowBuffer: Buffer): number` — Parse 4-byte accumulated encoder

**Layer 4: Object-Oriented API**
- `MotorChannel` class:
  - Constructor takes channel number (1-4) and board reference
  - Config: `setMotorType()`, `setDeadZone()`, `setReductionRatio()`, `setPulsesPerLine()`, `setWheelDiameter()`
  - Control: `setSpeed()`, `setPWM()`
  - Reading: `readRealtimeEncoder()`, `readAccumulatedEncoder()`
  - Each method uses Layer 1 builders + Layer 2 transport
  
- `QuadEncoderBoard` class (singleton):
  - Manages 4 `MotorChannel` instances
  - `channel(num: MotorChannel): MotorChannel` — Access specific channel
  - `readAllRealtimeEncoders(): number[]` — Batch read 0x10-0x13
  - `readAllAccumulatedEncoders(): number[]` — Batch read 0x20-0x27
  - `stopAll(): void` — Send zero speed to all motors

## Implementation Steps

### Step 1: Create register constants and enums
In `main.ts`:
- Define `MotorType`, `MotorChannel`, `EncoderMode` enums with block annotations
- Define I2C register addresses (0x01-0x07, 0x10-0x13, 0x20-0x27)
- I2C address constant (0x26)

### Step 2: Implement Layer 1 - Buffer builders
Create 20-30 pure functions:
- Each function takes typed params, returns `Buffer`
- Use `pins.createBuffer()` and bit manipulation
- Handle endianness correctly (big for 16-bit, little for float)
- No I2C communication in these functions

### Step 3: Implement Layer 2 - I2C transport
Create 2 functions:
- `writeI2C()` uses `pins.i2cWriteNumber()` or `pins.i2cWriteBuffer()`
- `readI2C()` uses `pins.i2cReadBuffer()`
- Add 100ms `basic.pause()` after config writes (EEPROM persistence)

### Step 4: Implement Layer 3 - Response parsers
Create 2-3 functions:
- Handle sign extension for 16-bit values (check 0x8000 bit)
- Combine high/low registers for 32-bit values
- Check for negative values (>= 0x80000000)

### Step 5: Implement MotorChannel class
- Private `_channelNum` property
- All methods delegate to Layers 1-3
- Channel-specific register offsets calculated internally
- Methods: configuration, control, reading

### Step 6: Implement QuadEncoderBoard class
- Create 4 `MotorChannel` instances in constructor
- Provide convenience methods for multi-channel operations
- Singleton pattern (one board per micro:bit)

### Step 7: Write test.ts hardware verification
Test sequence:
1. Initialize board with Motor Type 1 (520 motor) config
2. Ramp M1 speed: 0 → 500 → 1000 → 0 over 10 seconds
3. Log realtime encoder every second
4. Ramp M2-M4 in sequence with different patterns
5. Final test: All motors forward 500 for 2s, stop, read accumulated encoders
6. Verify encoder values make sense relative to speed/time

## Design Decisions to Resolve

1. **Configuration persistence**: Should we create a `configureMotorType(channel, type)` helper that sets all 5 parameters at once from the motor type table (deadzone, ratio, pulses, diameter)? Or keep granular setters?

2. **Error handling**: MakeCode has limited exception support — should buffer builders clamp values (e.g., speed -1000 to +1000) or assume valid inputs?

3. **Block annotations timing**: Add after Layer 4 is tested, or incrementally as we build each class?

4. **Motor type presets**: Should we expose the motor configuration table from docs/IIC.py as a const lookup table?

## Motor Configuration Reference

From docs/IIC.py - parameters for each motor type:

| Motor Type | Dead Zone | Reduction Ratio | Pulses/Line | Wheel Diameter | PWM Only |
|------------|-----------|-----------------|-------------|----------------|----------|
| 520 motor | 1600 | 30 | 11 | 67.00mm | No |
| 310 motor | 1200 | 20 | 13 | 48.00mm | No |
| TT encoder | 1250 | 45 | 13 | 68.00mm | No |
| TT DC geared | 1000 | 48 | N/A | N/A | **Yes** |
| L-type 520 | 1600 | 40 | 11 | 67.00mm | No |

**Note**: Type 4 (TT DC geared) has no encoder - use `setPWM()` only, not `setSpeed()`

## I2C Protocol Reference

**Address**: 0x26

**Registers**:
- Config: 0x01 (type), 0x02 (deadzone), 0x03 (pulses/line), 0x04 (ratio), 0x05 (wheel dia)
- Control: 0x06 (speed), 0x07 (PWM)
- Realtime encoders: 0x10-0x13 (2 bytes each)
- Accumulated encoders: 0x20-0x27 (4 bytes each, high/low split)

**Encoding**:
- 16-bit signed: Big-endian, 2's complement
- 32-bit signed: Combine high (2 bytes) << 16 | low (2 bytes)
- Float (wheel dia): IEEE 754 little-endian
