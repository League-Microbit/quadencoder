# Copilot Instructions for QuadEncoder Project

## Project Overview
This project creates a **micro:bit MakeCode extension** for the Yahboom 4-channel quad encoder motor controller. The goal is to translate the Python reference implementation ([docs/IIC.py](../docs/IIC.py)) into TypeScript that exposes a block-based interface in MakeCode.

**Hardware**: Yahboom 4-port motor controller board  
**Reference**: https://www.yahboom.net/public/upload/upload-html/1740571339/Drive%20motor%20and%20read%20encoder-USART.html  
**Communication**: I2C (address `0x26`) from micro:bit to motor controller

## Implementation Architecture

### Class Structure
The TypeScript implementation should follow this object-oriented design:

1. **`QuadEncoderBoard` class**: Represents the entire 4-channel motor controller
   - Handles I2C communication to address `0x26`
   - Manages 4 motor channels (M1-M4)
   - Provides methods for reading all encoders at once
   - Singleton pattern (one board per micro:bit)

2. **`MotorChannel` class**: Represents individual motor channels
   - Channel-specific configuration (motor type, dead zone, reduction ratio, wheel diameter)
   - Speed/PWM control for that channel
   - Encoder reading for that channel
   - Owned by parent `QuadEncoderBoard`

### Required Enumerations
Create TypeScript enums for MakeCode dropdown menus:

```typescript
enum MotorType {
    //% block="520 motor"
    Motor520 = 1,
    //% block="310 motor"
    Motor310 = 2,
    //% block="TT motor with encoder"
    TTMotorEncoder = 3,
    //% block="TT DC geared motor"
    TTMotorDC = 4,
    //% block="L-type 520 motor"
    LMotor520 = 5
}

enum EncoderMode {
    //% block="accumulated"
    Accumulated = 1,
    //% block="real-time"
    RealTime = 2
}

enum MotorChannel {
    //% block="M1"
    M1 = 1,
    //% block="M2"
    M2 = 2,
    //% block="M3"
    M3 = 3,
    //% block="M4"
    M4 = 4
}
```

### Motor Controller Capabilities
The board supports:
- **5 motor types**: 520, 310, TT with encoder disk, TT DC geared, L-type 520
- **Speed control**: Via encoder feedback (motors 1-3, 5) or direct PWM (motor 4)
- **Encoder reading**: Real-time (10ms interval) and accumulated counts
- **Persistent configuration**: Motor parameters saved to EEPROM on controller board

## Development Workflows

### Build & Deploy
```bash
make build    # Compile the extension
make deploy   # Deploy to connected micro:bit
make test     # Run tests
```

Or use PXT CLI directly:
```bash
pxt build
pxt deploy
pxt test
```

### Testing in MakeCode
1. Import from URL: `https://github.com/ericbusboom/quadencoder`
2. Or search for extension in MakeCode editor

### Publishing
The project uses GitHub Pages for extension hosting via Jekyll (`_config.yml`, `Gemfile`).

## Development Roadmap

### Phase 1: Core Classes (Without Block Annotations)
1. Create `MotorChannel` class with:
   - Configuration methods: `setMotorType()`, `setDeadZone()`, `setReductionRatio()`, `setWheelDiameter()`
   - Control methods: `setSpeed()`, `setPWM()`
   - Reading methods: `readEncoder()`, `readAccumulatedEncoder()`

2. Create `QuadEncoderBoard` class with:
   - Channel accessors: `channel(num: MotorChannel)`
   - Board-level methods: `readAllEncoders()`, `stopAll()`
   - I2C helper methods: `writeRegister()`, `readRegister()`

### Phase 2: Testing
1. Write tests in [test.ts](../test.ts) to verify:
   - I2C communication works correctly
   - Motor configuration persists
   - Speed control responds appropriately
   - Encoder readings are accurate
Motor Type Configuration Reference

Each motor type requires specific configuration values (from [docs/IIC.py](../docs/IIC.py)):

| Motor Type | Dead Zone | Reduction Ratio | Pulses/Line | Wheel Diameter | PWM Only |
|------------|-----------|-----------------|-------------|----------------|----------|
| 520 motor | 1600 | 30 | 11 | 67.00mm | No |
| 310 motor | 1200 | 20 | 13 | 48.00mm | No |
| TT encoder | 1250 | 45 | 13 | 68.00mm | No |
| TT DC geared | 1000 | 48 | N/A | N/A | **Yes** |
| L-type 520 | 1600 | 40 | 11 | 67.00mm | No |

**Critical Notes**:
- Type 4 (TT DC geared): Has no encoder, use `setPWM()` only, not `setSpeed()`
- Dead zone values prevent motor stalling at low speeds
- Configuration persists in EEPROM - set once per motor change
- Wheel diameter used for speed calculations (mm/s to encoder pulses)

## MakeCode Extension Conventions

### TypeScript Requirements
- **Target**: ES5 (micro:bit v1/v2 compatibility)
- **APIs**: Use `pins.i2cWriteBuffer()`, `pins.createBuffer()`, `basic.pause()`
- **No external libraries**: Everything must work in MakeCode sandbox

### Block Annotation Examples
```typescript
//% blockNamespace=QuadEncoder color="#AA278D" icon="\uf1b9"
//% block="motor $channel set speed to $speed"
//% channel.defl=MotorChannel.M1
//% speed.min=-1000 speed.max=1000 speed.defl=0
//% weight=100 blockGap=8
//% group="Motor Control"
```

Key directives:
- `blockNamespace`: Category name in toolbox
- `color`/`icon`: Visual appearance (use FontAwesome codes)
- `min`/`max`/`defl`: Slider constraints and defaults
- `weight`: Higher = appears first in toolbox
- `group`: Subcategory within namespace
- `blockGap`: Spacing after block

### File Structure
- [main.ts](../main.ts): All classes, enums, and exported functions
- [test.ts](../test.ts): Hardware tests (not compiled in extension)
- [pxt.json](../pxt.json): Metadata (update description once complete)

## Build & Deploy

```bash
# Local development
pxt build          # Compile TypeScript
pxt deploy         # Flash to connected micro:bit
pxt test           # Run test.ts

# Or use Makefile shortcuts
make build
make deploy
make test
```

### Testing in MakeCode Web Editor
1. Go to https://makecode.microbit.org/
2. Click Extensions → Import URL
3. Enter: `https://github.com/ericbusboom/quadencoder`
4. Blocks appear in toolbox under "QuadEncoder" category

## Next Actions
Start with Phase 1: Create `MotorChannel` and `QuadEncoderBoard` classes without block annotations, focusing on correct I2C communication and data encoding.
- `//% block`: Define block text with $parameter placeholders
- `//% defl`: Set default values
- `//% weight`: Control block ordering in toolbox
- `//% blockId`: Unique identifier for the block
- `//% group`: Group related blocks together

## I2C Protocol Details

### Register Map (from docs/IIC.py)
```
Configuration Registers:
0x01: MOTOR_TYPE_REG (1 byte)
0x02: MOTOR_DEADZONE_REG (2 bytes, big-endian)
0x03: MOTOR_PLUSELINE_REG (2 bytes, big-endian) - encoder pulses per line
0x04: MOTOR_PLUSEPHASE_REG (2 bytes, big-endian) - reduction ratio
0x05: WHEEL_DIA_REG (4 bytes, little-endian float)

Control Registers:
0x06: SPEED_CONTROL_REG (8 bytes: 4 motors × 2 bytes each)
0x07: PWM_CONTROL_REG (8 bytes: 4 motors × 2 bytes each)

Encoder Reading Registers:
0x10-0x13: READ_TEN_M1-M4_ENCODER_REG (2 bytes each, real-time)
0x20-0x27: READ_ALLHIGH/LOW_M1-M4_REG (4 bytes per motor, accumulated)
```

### Data Encoding Rules
- **16-bit signed integers** (speed, PWM, real-time encoder): Big-endian, 2's complement
  - Positive: 0x0000 to 0x7FFF
  - Negative: 0x8000 to 0xFFFF
- **32-bit signed integers** (accumulated encoder): Combine high (0x20) and low (0x21) registers
  - Read high 16 bits, shift left 16, OR with low 16 bits
  - Check if >= 0x80000000 for negative values
- **Floating point** (wheel diameter): 4 bytes, IEEE 754 little-endian
  - Use `pins.createBuffer()` and manual byte packing

### I2C Communication Pattern
```typescript
// Writing 2-byte value (big-endian)
let buf = pins.createBuffer(2)
buf[0] = (value >> 8) & 0xFF  // High byte
buf[1] = value & 0xFF          // Low byte
pins.i2cWriteBuffer(0x26, Buffer.concat([Buffer.fromArray([register]), buf]))

// Reading 2-byte value
let data = pins.i2cReadBuffer(0x26, 2, false)
let value = (data[0] << 8) | data[1]
if (value & 0x8000) value -= 0x10000  // Sign extend
```

**Important**: Add 100ms delays after configuration writes (EEPROM persistence)

## Key Implementation Notes
- **Motor dead zone**: Each motor type requires specific dead zone values (1000-1600) for reliable operation
- **Configuration persistence**: Motor parameters saved to controller EEPROM; only set once unless changing motors
- **Type 4 motors** (TT DC geared): Use PWM control only (no encoder feedback)
- **Delay after I2C writes**: Reference implementation uses 0.1s delays after configuration writes

## Next Steps for Implementation
The current [main.ts](../main.ts) is skeletal. To complete this extension:
1. Implement I2C communication functions matching the Python reference
2. Create MakeCode block definitions for motor control
3. Add encoder reading blocks
4. Add configuration blocks for motor parameters
5. Update [test.ts](../test.ts) with validation tests
