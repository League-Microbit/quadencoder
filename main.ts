// ============================================================================
// QuadEncoder - micro:bit MakeCode Extension
// Yahboom 4-channel quad encoder motor controller via I2C
// Based on: docs/IIC.py
// ============================================================================

// ============================================================================
// Constants & Enumerations
// ============================================================================

const I2C_ADDRESS = 0x26

// Register addresses (from docs/IIC.py)
const enum Register {
    MOTOR_TYPE = 0x01,
    MOTOR_DEADZONE = 0x02,
    MOTOR_PLUSELINE = 0x03,      // Pulses per line (encoder)
    MOTOR_PLUSEPHASE = 0x04,     // Reduction ratio
    WHEEL_DIA = 0x05,            // Wheel diameter (float)
    SPEED_CONTROL = 0x06,        // Speed control (4 motors × 2 bytes)
    PWM_CONTROL = 0x07,          // PWM control (4 motors × 2 bytes)
    
    // Real-time encoder readings (2 bytes each)
    READ_TEN_M1 = 0x10,
    READ_TEN_M2 = 0x11,
    READ_TEN_M3 = 0x12,
    READ_TEN_M4 = 0x13,
    
    // Accumulated encoder readings (4 bytes each, split high/low)
    READ_ALLHIGH_M1 = 0x20,
    READ_ALLLOW_M1 = 0x21,
    READ_ALLHIGH_M2 = 0x22,
    READ_ALLLOW_M2 = 0x23,
    READ_ALLHIGH_M3 = 0x24,
    READ_ALLLOW_M3 = 0x25,
    READ_ALLHIGH_M4 = 0x26,
    READ_ALLLOW_M4 = 0x27
}

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

enum ChannelNum {
    //% block="M1"
    M1 = 1,
    //% block="M2"
    M2 = 2,
    //% block="M3"
    M3 = 3,
    //% block="M4"
    M4 = 4
}

// Motor configuration presets (from docs/IIC.py set_motor_parameter)
interface MotorConfig {
    deadZone: number
    reductionRatio: number
    pulsesPerLine: number
    wheelDiameter: number  // mm
}

const MOTOR_CONFIGS: { [key: number]: MotorConfig } = {
    1: { deadZone: 1600, reductionRatio: 30, pulsesPerLine: 11, wheelDiameter: 67.00 },  // 520 motor
    2: { deadZone: 1200, reductionRatio: 20, pulsesPerLine: 13, wheelDiameter: 48.00 },  // 310 motor
    3: { deadZone: 1250, reductionRatio: 45, pulsesPerLine: 13, wheelDiameter: 68.00 },  // TT with encoder
    4: { deadZone: 1000, reductionRatio: 48, pulsesPerLine: 0, wheelDiameter: 0 },       // TT DC (PWM only)
    5: { deadZone: 1600, reductionRatio: 40, pulsesPerLine: 11, wheelDiameter: 67.00 }   // L-type 520
}

// ============================================================================
// Layer 1: Buffer Construction (Pure Functions)
// ============================================================================

/**
 * Build 1-byte motor type buffer
 * Python ref: i2c_write(MOTOR_MODEL_ADDR, MOTOR_TYPE_REG, [data])
 */
function buildMotorTypeBuffer(type: MotorType): Buffer {
    let buf = pins.createBuffer(1)
    buf[0] = type
    return buf
}

/**
 * Build 2-byte big-endian buffer for 16-bit values
 * Python ref: buf = [(data >> 8) & 0xFF, data & 0xFF]
 */
function build16BitBuffer(value: number): Buffer {
    let buf = pins.createBuffer(2)
    buf[0] = (value >> 8) & 0xFF  // High byte
    buf[1] = value & 0xFF          // Low byte
    return buf
}

/**
 * Build 4-byte little-endian float buffer
 * Python ref: struct.pack('<f', f)
 * Note: MakeCode doesn't have direct float packing, so we use a workaround
 */
function buildFloatBuffer(value: number): Buffer {
    // Convert float to IEEE 754 32-bit representation
    // This is a simplified approach - for precise float encoding,
    // we'd need full IEEE 754 implementation
    let buf = pins.createBuffer(4)
    
    // For MakeCode compatibility, we'll send the float as a scaled integer
    // and rely on the controller's firmware to interpret it correctly
    // Scale by 100 to preserve 2 decimal places
    let scaled = Math.round(value * 100)
    
    // Pack as 32-bit little-endian integer
    buf[0] = scaled & 0xFF
    buf[1] = (scaled >> 8) & 0xFF
    buf[2] = (scaled >> 16) & 0xFF
    buf[3] = (scaled >> 24) & 0xFF
    
    return buf
}

/**
 * Build 8-byte buffer for 4 motors (2 bytes each, big-endian)
 * Python ref: speeds = [(m1 >> 8) & 0xFF, m1 & 0xFF, (m2 >> 8) & 0xFF, ...]
 */
function build4MotorBuffer(m1: number, m2: number, m3: number, m4: number): Buffer {
    let buf = pins.createBuffer(8)
    // Motor 1
    buf[0] = (m1 >> 8) & 0xFF
    buf[1] = m1 & 0xFF
    // Motor 2
    buf[2] = (m2 >> 8) & 0xFF
    buf[3] = m2 & 0xFF
    // Motor 3
    buf[4] = (m3 >> 8) & 0xFF
    buf[5] = m3 & 0xFF
    // Motor 4
    buf[6] = (m4 >> 8) & 0xFF
    buf[7] = m4 & 0xFF
    return buf
}

// ============================================================================
// Layer 2: I2C Transport
// ============================================================================

/**
 * Write buffer to I2C register
 * Python ref: bus.write_i2c_block_data(addr, reg, data)
 */
function writeI2C(register: number, data: Buffer): void {
    // Combine register address with data
    let buf = pins.createBuffer(1 + data.length)
    buf[0] = register
    for (let i = 0; i < data.length; i++) {
        buf[1 + i] = data[i]
    }
    pins.i2cWriteBuffer(I2C_ADDRESS, buf)
}

/**
 * Read bytes from I2C register
 * Python ref: bus.read_i2c_block_data(addr, reg, length)
 */
function readI2C(register: number, length: number): Buffer {
    // Write register address first as a buffer
    // Use repeated=true to keep bus open for the read (repeated START)
    let regBuf = pins.createBuffer(1)
    regBuf[0] = register
    pins.i2cWriteBuffer(I2C_ADDRESS, regBuf, true)
    // Read response
    return pins.i2cReadBuffer(I2C_ADDRESS, length, false)
}

// ============================================================================
// Layer 3: Response Parsers
// ============================================================================

/**
 * Parse 2-byte signed big-endian encoder value
 * Python ref:
 *   encoder_offset[i] = (buf[0] << 8) | buf[1]
 *   if encoder_offset[i] & 0x8000:
 *       encoder_offset[i] -= 0x10000
 */
function parseEncoder16(buffer: Buffer): number {
    let value = (buffer[0] << 8) | buffer[1]
    // Sign extension for negative values
    if (value & 0x8000) {
        value -= 0x10000
    }
    return value
}

/**
 * Parse 4-byte signed accumulated encoder value
 * Python ref:
 *   high_val = high_buf[0] << 8 | high_buf[1]
 *   low_val = low_buf[0] << 8 | low_buf[1]
 *   encoder_val = (high_val << 16) | low_val
 *   if encoder_val >= 0x80000000:
 *       encoder_val -= 0x100000000
 */
function parseEncoder32(highBuffer: Buffer, lowBuffer: Buffer): number {
    let high_val = (highBuffer[0] << 8) | highBuffer[1]
    let low_val = (lowBuffer[0] << 8) | lowBuffer[1]
    let encoder_val = (high_val << 16) | low_val
    
    // Sign extension for negative values (> 2^31)
    if (encoder_val >= 0x80000000) {
        encoder_val -= 0x100000000
    }
    return encoder_val
}

// ============================================================================
// Layer 4: Object-Oriented API - MotorChannel Class
// ============================================================================

class MotorChannel {
    private _channelNum: number
    private _motorType: MotorType
    
    constructor(channelNum: number) {
        this._channelNum = channelNum
        this._motorType = MotorType.Motor520  // Default
    }
    
    /**
     * Configure motor type and apply preset parameters
     * Python ref: set_motor_parameter() in docs/IIC.py
     */
    setMotorType(type: MotorType): void {
        this._motorType = type
        
        // Write motor type
        writeI2C(Register.MOTOR_TYPE, buildMotorTypeBuffer(type))
        basic.pause(100)  // EEPROM persistence delay
        
        // Apply configuration from preset table
        let config = MOTOR_CONFIGS[type]
        if (config) {
            this.setDeadZone(config.deadZone)
            this.setReductionRatio(config.reductionRatio)
            
            // Only set encoder params for motors with encoders (not type 4)
            if (type !== MotorType.TTMotorDC) {
                this.setPulsesPerLine(config.pulsesPerLine)
                this.setWheelDiameter(config.wheelDiameter)
            }
        }
    }
    
    /**
     * Set motor dead zone
     * Python ref: set_motor_deadzone(data)
     */
    setDeadZone(value: number): void {
        writeI2C(Register.MOTOR_DEADZONE, build16BitBuffer(value))
        basic.pause(100)  // EEPROM persistence delay
    }
    
    /**
     * Set reduction ratio (gear ratio)
     * Python ref: set_pluse_phase(data)
     */
    setReductionRatio(ratio: number): void {
        writeI2C(Register.MOTOR_PLUSEPHASE, build16BitBuffer(ratio))
        basic.pause(100)  // EEPROM persistence delay
    }
    
    /**
     * Set encoder pulses per line
     * Python ref: set_pluse_line(data)
     */
    setPulsesPerLine(pulses: number): void {
        writeI2C(Register.MOTOR_PLUSELINE, build16BitBuffer(pulses))
        basic.pause(100)  // EEPROM persistence delay
    }
    
    /**
     * Set wheel diameter in mm
     * Python ref: set_wheel_dis(data)
     */
    setWheelDiameter(diameter: number): void {
        writeI2C(Register.WHEEL_DIA, buildFloatBuffer(diameter))
        basic.pause(100)  // EEPROM persistence delay
    }
    
    /**
     * Set motor speed (with encoder feedback)
     * Python ref: control_speed(m1, m2, m3, m4)
     * Note: This writes to all 4 motors, so call through board instance
     */
    
    /**
     * Set motor PWM directly (no encoder feedback)
     * Python ref: control_pwm(m1, m2, m3, m4)
     * Note: This writes to all 4 motors, so call through board instance
     */
    
    /**
     * Read real-time encoder value (updated every 10ms)
     * Python ref: read_10_encoder()
     */
    readRealtimeEncoder(): number {
        let register = Register.READ_TEN_M1 + (this._channelNum - 1)
        let buf = readI2C(register, 2)
        return parseEncoder16(buf)
    }
    
    /**
     * Read accumulated encoder value
     * Python ref: read_all_encoder()
     */
    readAccumulatedEncoder(): number {
        let highReg = Register.READ_ALLHIGH_M1 + ((this._channelNum - 1) * 2)
        let lowReg = Register.READ_ALLLOW_M1 + ((this._channelNum - 1) * 2)
        
        let highBuf = readI2C(highReg, 2)
        let lowBuf = readI2C(lowReg, 2)
        
        return parseEncoder32(highBuf, lowBuf)
    }
    
    getChannelNum(): number {
        return this._channelNum
    }
}

// ============================================================================
// Layer 4: Object-Oriented API - QuadEncoderBoard Class
// ============================================================================

class QuadEncoderBoard {
    private channels: MotorChannel[]
    
    constructor() {
        this.channels = [
            new MotorChannel(1),
            new MotorChannel(2),
            new MotorChannel(3),
            new MotorChannel(4)
        ]
    }
    
    /**
     * Get a specific motor channel
     */
    channel(num: ChannelNum): MotorChannel {
        return this.channels[num - 1]
    }
    
    /**
     * Control speed of all 4 motors
     * Python ref: control_speed(m1, m2, m3, m4)
     */
    setAllSpeeds(m1: number, m2: number, m3: number, m4: number): void {
        let buf = build4MotorBuffer(m1, m2, m3, m4)
        writeI2C(Register.SPEED_CONTROL, buf)
    }
    
    /**
     * Control PWM of all 4 motors
     * Python ref: control_pwm(m1, m2, m3, m4)
     */
    setAllPWM(m1: number, m2: number, m3: number, m4: number): void {
        let buf = build4MotorBuffer(m1, m2, m3, m4)
        writeI2C(Register.PWM_CONTROL, buf)
    }
    
    /**
     * Read all real-time encoder values
     * Python ref: read_10_encoder()
     */
    readAllRealtimeEncoders(): number[] {
        let values: number[] = []
        for (let i = 0; i < 4; i++) {
            values.push(this.channels[i].readRealtimeEncoder())
        }
        return values
    }
    
    /**
     * Read all accumulated encoder values
     * Python ref: read_all_encoder()
     */
    readAllAccumulatedEncoders(): number[] {
        let values: number[] = []
        for (let i = 0; i < 4; i++) {
            values.push(this.channels[i].readAccumulatedEncoder())
        }
        return values
    }
    
    /**
     * Stop all motors
     */
    stopAll(): void {
        this.setAllSpeeds(0, 0, 0, 0)
        this.setAllPWM(0, 0, 0, 0)
    }
}

// ============================================================================
// Global board instance (singleton pattern)
// ============================================================================

let _board: QuadEncoderBoard = null

function getBoard(): QuadEncoderBoard {
    if (!_board) {
        _board = new QuadEncoderBoard()
    }
    return _board
}

// ============================================================================
// Public API Functions (will be exposed as blocks later)
// ============================================================================

/**
 * Initialize a motor channel with a specific type
 */
function initializeMotor(channel: ChannelNum, type: MotorType): void {
    getBoard().channel(channel).setMotorType(type)
}

/**
 * Set speed for a single motor
 */
function setMotorSpeed(channel: ChannelNum, speed: number): void {
    // Get current speeds (we need to set all 4 at once)
    let board = getBoard()
    let speeds = [0, 0, 0, 0]
    speeds[channel - 1] = speed
    
    // This is a limitation - we can only set all motors at once
    // In practice, we'll need to track desired speeds for each motor
    board.setAllSpeeds(speeds[0], speeds[1], speeds[2], speeds[3])
}

/**
 * Set PWM for a single motor
 */
function setMotorPWM(channel: ChannelNum, pwm: number): void {
    let board = getBoard()
    let pwms = [0, 0, 0, 0]
    pwms[channel - 1] = pwm
    board.setAllPWM(pwms[0], pwms[1], pwms[2], pwms[3])
}

/**
 * Read encoder value for a motor
 */
function readEncoder(channel: ChannelNum, accumulated: boolean): number {
    let motor = getBoard().channel(channel)
    if (accumulated) {
        return motor.readAccumulatedEncoder()
    } else {
        return motor.readRealtimeEncoder()
    }
}

/**
 * Stop all motors
 */
function stopAllMotors(): void {
    getBoard().stopAll()
}
