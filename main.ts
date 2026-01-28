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

// ============================================================================
// Position Classes for Target Specification
// ============================================================================

/**
 * Base class for position specifications
 */
class Position {
    protected _value: number
    protected _isRelative: boolean
    
    constructor(value: number, isRelative: boolean) {
        this._value = value
        this._isRelative = isRelative
    }
    
    /**
     * Convert position to absolute encoder counts
     * @param currentPosition Current encoder position in counts
     * @param pulsesPerRevolution Total encoder pulses per full revolution
     */
    toEncoderCounts(currentPosition: number, pulsesPerRevolution: number): number {
        return 0  // Override in subclasses
    }
    
    isRelative(): boolean {
        return this._isRelative
    }
}

/**
 * Position specified in encoder counts
 */
class EncoderCountsPosition extends Position {
    constructor(counts: number, isRelative: boolean = false) {
        super(counts, isRelative)
    }
    
    toEncoderCounts(currentPosition: number, pulsesPerRevolution: number): number {
        if (this._isRelative) {
            return currentPosition + this._value
        }
        return this._value
    }
    
    /**
     * Create absolute position in encoder counts
     */
    static absolute(counts: number): EncoderCountsPosition {
        return new EncoderCountsPosition(counts, false)
    }
    
    /**
     * Create relative position in encoder counts
     */
    static relative(counts: number): EncoderCountsPosition {
        return new EncoderCountsPosition(counts, true)
    }
}

/**
 * Position specified in degrees
 */
class DegreesPosition extends Position {
    constructor(degrees: number, isRelative: boolean = false) {
        super(degrees, isRelative)
    }
    
    toEncoderCounts(currentPosition: number, pulsesPerRevolution: number): number {
        // Convert degrees to encoder counts
        // 360 degrees = pulsesPerRevolution counts
        let counts = Math.round((this._value / 360.0) * pulsesPerRevolution)
        
        if (this._isRelative) {
            return currentPosition + counts
        }
        return counts
    }
    
    /**
     * Create absolute position in degrees
     */
    static absolute(degrees: number): DegreesPosition {
        return new DegreesPosition(degrees, false)
    }
    
    /**
     * Create relative position in degrees
     */
    static relative(degrees: number): DegreesPosition {
        return new DegreesPosition(degrees, true)
    }
}

// ============================================================================
// PID Controller
// ============================================================================

class PIDController {
    // PID gains
    private _kP: number
    private _kI: number
    private _kD: number
    
    // Limits
    private _maxAcceleration: number  // Encoder counts per second^2
    private _maxSpeed: number         // Maximum speed limit
    private _tolerance: number        // Position tolerance in encoder counts
    
    // State
    private _target: number
    private _integral: number
    private _lastError: number
    private _lastTime: number
    private _active: boolean
    private _currentSpeed: number
    
    constructor(
        kP: number = 2.0,
        kI: number = 0.1,
        kD: number = 0.5,
        maxAcceleration: number = 500,
        maxSpeed: number = 1000,
        tolerance: number = 10
    ) {
        this._kP = kP
        this._kI = kI
        this._kD = kD
        this._maxAcceleration = maxAcceleration
        this._maxSpeed = maxSpeed
        this._tolerance = tolerance
        
        // Initialize state
        this._target = 0
        this._integral = 0
        this._lastError = 0
        this._lastTime = 0
        this._active = false
        this._currentSpeed = 0
    }
    
    /**
     * Set PID gains
     */
    setGains(kP: number, kI: number, kD: number): void {
        this._kP = kP
        this._kI = kI
        this._kD = kD
    }
    
    /**
     * Set maximum acceleration (encoder counts per second squared)
     */
    setMaxAcceleration(accel: number): void {
        this._maxAcceleration = accel
    }
    
    /**
     * Set maximum speed limit
     */
    setMaxSpeed(speed: number): void {
        this._maxSpeed = speed
    }
    
    /**
     * Set position tolerance (encoder counts)
     */
    setTolerance(tolerance: number): void {
        this._tolerance = tolerance
    }
    
    /**
     * Set target position and activate controller
     */
    setTarget(target: number): void {
        this._target = target
        this._integral = 0
        this._lastError = 0
        this._lastTime = control.millis()
        this._active = true
    }
    
    /**
     * Check if controller is active
     */
    isActive(): boolean {
        return this._active
    }
    
    /**
     * Stop the controller and reset speed
     */
    stop(): void {
        this._active = false
        this._currentSpeed = 0
    }
    
    /**
     * Update PID control loop
     * @param currentPosition Current encoder position
     * @returns Object with speed command and whether target is reached
     */
    update(currentPosition: number): { speed: number, atTarget: boolean } {
        if (!this._active) {
            return { speed: 0, atTarget: true }
        }
        
        // Calculate error
        let error = this._target - currentPosition
        
        // Check if we've reached the target
        if (Math.abs(error) <= this._tolerance) {
            this._active = false
            this._currentSpeed = 0
            return { speed: 0, atTarget: true }
        }
        
        // Calculate time delta
        let currentTime = control.millis()
        let dt = (currentTime - this._lastTime) / 1000.0  // Convert to seconds
        if (dt <= 0) dt = 0.01  // Prevent division by zero
        
        // PID calculations
        this._integral += error * dt
        
        // Anti-windup: clamp integral
        let maxIntegral = 1000
        if (this._integral > maxIntegral) this._integral = maxIntegral
        if (this._integral < -maxIntegral) this._integral = -maxIntegral
        
        let derivative = (error - this._lastError) / dt
        
        // Calculate desired speed from PID
        let desiredSpeed = this._kP * error + this._kI * this._integral + this._kD * derivative
        
        // Apply acceleration limits
        let maxSpeedChange = this._maxAcceleration * dt
        let speedChange = desiredSpeed - this._currentSpeed
        
        if (speedChange > maxSpeedChange) {
            speedChange = maxSpeedChange
        } else if (speedChange < -maxSpeedChange) {
            speedChange = -maxSpeedChange
        }
        
        this._currentSpeed += speedChange
        
        // Apply speed limits
        if (this._currentSpeed > this._maxSpeed) this._currentSpeed = this._maxSpeed
        if (this._currentSpeed < -this._maxSpeed) this._currentSpeed = -this._maxSpeed
        
        // Update state
        this._lastError = error
        this._lastTime = currentTime
        
        return { speed: Math.round(this._currentSpeed), atTarget: false }
    }
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
 * Manual IEEE 754 float encoding since MakeCode lacks struct.pack
 */
function buildFloatBuffer(value: number): Buffer {
    let buf = pins.createBuffer(4)
    
    // Handle special cases
    if (value === 0) {
        // Positive zero: all bytes 0x00
        return buf
    }
    
    // IEEE 754 single precision format:
    // [sign: 1 bit][exponent: 8 bits][mantissa: 23 bits]
    
    let sign = 0
    if (value < 0) {
        sign = 1
        value = -value
    }
    
    // Calculate exponent and mantissa
    let exponent = 0
    let mantissa = value
    
    // Normalize: adjust mantissa to range [1.0, 2.0)
    while (mantissa >= 2.0) {
        mantissa /= 2.0
        exponent++
    }
    while (mantissa < 1.0) {
        mantissa *= 2.0
        exponent--
    }
    
    // Remove implicit leading 1 and convert to 23-bit integer
    mantissa = (mantissa - 1.0) * 0x800000  // 2^23
    
    // Add bias to exponent (127 for single precision)
    exponent += 127
    
    // Clamp exponent to valid range
    if (exponent <= 0) {
        exponent = 0
        mantissa = 0
    } else if (exponent >= 255) {
        exponent = 255
        mantissa = 0
    }
    
    // Pack into 32-bit value (little-endian)
    let mantissaInt = Math.floor(mantissa)
    let byte0 = mantissaInt & 0xFF
    let byte1 = (mantissaInt >> 8) & 0xFF
    let byte2 = ((mantissaInt >> 16) & 0x7F) | ((exponent & 0x01) << 7)
    let byte3 = ((exponent >> 1) & 0x7F) | (sign << 7)
    
    buf[0] = byte0
    buf[1] = byte1
    buf[2] = byte2
    buf[3] = byte3
    
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
    private _pulsesPerRevolution: number
    
    // PID controller
    private _pidController: PIDController
    
    constructor(
        channelNum: number, 
        motorType: MotorType, 
        deadZone: number, 
        reductionRatio: number, 
        pulsesPerLine: number = 0, 
        wheelDiameter: number = 0,
        kP: number = 2.0,
        kI: number = 0.1,
        kD: number = 0.5,
        maxAcceleration: number = 500,  // counts/s^2
        maxSpeed: number = 1000
    ) {
        this._channelNum = channelNum
        this._motorType = motorType
        
        // Calculate total pulses per revolution (pulses per line * reduction ratio * 4 for quadrature)
        this._pulsesPerRevolution = pulsesPerLine * reductionRatio * 4
        
        // Initialize PID controller
        this._pidController = new PIDController(kP, kI, kD, maxAcceleration, maxSpeed)
        
        // Write all configuration to board
        writeI2C(Register.MOTOR_TYPE, buildMotorTypeBuffer(motorType))
        basic.pause(100)
        
        this.setDeadZone(deadZone)
        this.setReductionRatio(reductionRatio)
        
        // Only set encoder params for motors with encoders (not type 4)
        if (motorType !== MotorType.TTMotorDC && pulsesPerLine > 0) {
            this.setPulsesPerLine(pulsesPerLine)
            this.setWheelDiameter(wheelDiameter)
        }
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
    
    /**
     * Set PID gains
     */
    setPID(kP: number, kI: number, kD: number): void {
        this._pidController.setGains(kP, kI, kD)
    }
    
    /**
     * Set maximum acceleration (encoder counts per second squared)
     */
    setMaxAcceleration(accel: number): void {
        this._pidController.setMaxAcceleration(accel)
    }
    
    /**
     * Set maximum speed limit
     */
    setMaxSpeed(speed: number): void {
        this._pidController.setMaxSpeed(speed)
    }
    
    /**
     * Set position tolerance for position control (encoder counts)
     */
    setTolerance(tolerance: number): void {
        this._pidController.setTolerance(tolerance)
    }
    
    /**
     * Update PID control loop (call periodically for background position control)
     * Returns true if at target position
     */
    updatePID(): boolean {
        let currentPosition = this.readAccumulatedEncoder()
        let result = this._pidController.update(currentPosition)
        
        if (result.speed !== 0 || !result.atTarget) {
            this._setSpeedInternal(result.speed)
        }
        
        return result.atTarget
    }
    
    /**
     * Internal method to set speed for a single motor
     */
    private _setSpeedInternal(speed: number): void {
        // This is a limitation - we can only set all motors at once
        // For now, we'll create a helper buffer and write directly
        let speeds = [0, 0, 0, 0]
        speeds[this._channelNum - 1] = speed
        let buf = build4MotorBuffer(speeds[0], speeds[1], speeds[2], speeds[3])
        writeI2C(Register.SPEED_CONTROL, buf)
    }
    
    /**
     * Move to target position (blocking)
     * Blocks until position is reached within tolerance
     * @param position Target position (EncoderCountsPosition or DegreesPosition)
     * @param timeout Maximum time to wait in milliseconds (0 = no timeout)
     */
    moveToPosition(position: Position, timeout: number = 0): boolean {
        // Convert position to encoder counts
        let currentPosition = this.readAccumulatedEncoder()
        let targetCounts = position.toEncoderCounts(currentPosition, this._pulsesPerRevolution)
        
        // Set target and activate PID controller
        this._pidController.setTarget(targetCounts)
        
        let startTime = control.millis()
        
        // Run control loop until target reached or timeout
        while (true) {
            let atTarget = this.updatePID()
            
            if (atTarget) {
                return true  // Successfully reached target
            }
            
            // Check timeout
            if (timeout > 0 && (control.millis() - startTime) >= timeout) {
                this._pidController.stop()
                this._setSpeedInternal(0)
                return false  // Timeout
            }
            
            basic.pause(20)  // 50Hz control loop
        }
    }
    
    /**
     * Move to target position (non-blocking)
     * Starts movement and returns immediately
     * Call updatePID() periodically to continue control
     * @param position Target position (EncoderCountsPosition or DegreesPosition)
     */
    moveToPositionAsync(position: Position): void {
        // Convert position to encoder counts
        let currentPosition = this.readAccumulatedEncoder()
        let targetCounts = position.toEncoderCounts(currentPosition, this._pulsesPerRevolution)
        
        // Set target and activate PID controller
        this._pidController.setTarget(targetCounts)
    }
    
    /**
     * Check if motor is currently moving to a position
     */
    isMoving(): boolean {
        return this._pidController.isActive()
    }
    
    /**
     * Stop position control and halt motor
     */
    stopPositionControl(): void {
        this._pidController.stop()
        this._setSpeedInternal(0)
    }
    
    /**
     * Factory method to create a motor channel from a preset motor type
     */
    static fromPreset(channelNum: number, type: MotorType, kP: number = 2.0, kI: number = 0.1, kD: number = 0.5, maxAccel: number = 500, maxSpeed: number = 1000): MotorChannel {
        let config = MOTOR_CONFIGS[type]
        if (config) {
            return new MotorChannel(
                channelNum,
                type,
                config.deadZone,
                config.reductionRatio,
                config.pulsesPerLine,
                config.wheelDiameter,
                kP,
                kI,
                kD,
                maxAccel,
                maxSpeed
            )
        }
        // Fallback to 520 motor if config not found
        let defaultConfig = MOTOR_CONFIGS[1]
        return new MotorChannel(
            channelNum,
            type,
            defaultConfig.deadZone,
            defaultConfig.reductionRatio,
            defaultConfig.pulsesPerLine,
            defaultConfig.wheelDiameter,
            kP,
            kI,
            kD,
            maxAccel,
            maxSpeed
        )
    }
}

// ============================================================================
// Layer 4: Object-Oriented API - QuadEncoderBoard Class
// ============================================================================

class QuadEncoderBoard {
    private channels: (MotorChannel | null)[]
    
    constructor(m1?: MotorChannel, m2?: MotorChannel, m3?: MotorChannel, m4?: MotorChannel) {
        this.channels = [
            m1 || null,
            m2 || null,
            m3 || null,
            m4 || null
        ]
    }
    
    /**
     * Register a motor channel with the board
     */
    registerChannel(channel: MotorChannel): void {
        let num = channel.getChannelNum()
        if (num >= 1 && num <= 4) {
            this.channels[num - 1] = channel
        }
    }
    
    /**
     * Get a specific motor channel
     */
    channel(num: ChannelNum): MotorChannel {
        let ch = this.channels[num - 1]
        if (!ch) {
            // Create a default channel if not registered
            ch = MotorChannel.fromPreset(num, MotorType.Motor520)
            this.channels[num - 1] = ch
        }
        return ch
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
