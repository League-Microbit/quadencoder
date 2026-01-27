// ============================================================================
// QuadEncoder Test Suite
// Hardware verification tests for motor controller
// ============================================================================

/**
 * Check if the I2C board is responding
 * Returns true if board responds, false otherwise
 */
function checkI2CConnection(): boolean {
    // Try to write register address as a proper I2C command
    // Using READ_TEN_M1 register (0x10) which should always be readable
    let regBuf = pins.createBuffer(1)
    regBuf[0] = 0x10
    pins.i2cWriteBuffer(0x26, regBuf, false)
    
    // Now try to read the response
    let devices = pins.i2cReadBuffer(0x26, 2, false)
    
    // If all bytes are 0xFF, likely no device present (pull-up resistors)
    if (devices[0] == 0xFF && devices[1] == 0xFF) {
        return false
    }
    
    // If we got here without error, board is connected
    // (0x00 0x00 is valid data - encoder at zero position)
    return true
}

/**
 * Wait for I2C connection to board with visual feedback
 * Displays "?" while checking, checkmark when connected, "X" if failed
 */
function waitForI2CConnection(): void {
    while (true) {
        // Show question mark while checking
        basic.showLeds(`
            . # # # .
            # . . . #
            . . . # .
            . . # . .
            . . # . .
            `)
        serial.writeLine("Checking I2C connection...")
        
        if (checkI2CConnection()) {
            // Board found - show checkmark
            basic.showIcon(IconNames.Yes)
            serial.writeLine("I2C board detected at 0x26!")
            basic.pause(1000)
            break
        } else {
            // Board not found - show X
            basic.showIcon(IconNames.No)
            serial.writeLine("I2C board not found, retrying...")
            basic.pause(1000)
        }
    }
}

/**
 * Test 1: Initialize and configure Motor 1 as 310 motor
 */
function test_initializeMotor(): void {
    serial.writeLine("=== Test 1: Initialize Motor ===")
    
    let board = getBoard()
    let motor1 = board.channel(ChannelNum.M1)
    
    serial.writeLine("Configuring M1 as 310 motor...")
    motor1.setMotorType(MotorType.Motor310)
    
    serial.writeLine("Motor 1 initialized")
    basic.pause(1000)
}

/**
 * Test 2: Speed ramp for Motor 1
 * Ramp: 0 -> 500 -> 1000 -> 500 -> 0
 */
function test_speedRamp(): void {
    serial.writeLine("=== Test 2: Speed Ramp M1 ===")
    
    let board = getBoard()
    let speeds = [0, 500, 1000, 500, 0]
    
    for (let speed of speeds) {
        serial.writeLine("Setting M1 speed: " + speed)
        board.setAllSpeeds(speed, 0, 0, 0)
        
        // Read encoder for 3 seconds
        for (let i = 0; i < 3; i++) {
            basic.pause(1000)
            let encoder = board.channel(ChannelNum.M1).readRealtimeEncoder()
            serial.writeLine("  Realtime encoder: " + encoder)
        }
    }
    
    board.stopAll()
    serial.writeLine("Motor stopped")
}

/**
 * Test 3: Read accumulated encoders
 */
function test_accumulatedEncoders(): void {
    serial.writeLine("=== Test 3: Accumulated Encoders ===")
    
    let board = getBoard()
    
    // Run motor at moderate speed for 5 seconds
    serial.writeLine("Running M1 at speed 500 for 5s...")
    board.setAllSpeeds(500, 0, 0, 0)
    basic.pause(5000)
    
    // Stop and read accumulated
    board.stopAll()
    let accumulated = board.channel(ChannelNum.M1).readAccumulatedEncoder()
    serial.writeLine("Accumulated encoder M1: " + accumulated)
    
    basic.pause(1000)
}

/**
 * Test 4: All motors sequence
 */
function test_allMotors(): void {
    serial.writeLine("=== Test 4: All Motors Test ===")
    
    let board = getBoard()
    
    // Initialize all motors as 310 type
    serial.writeLine("Initializing all motors...")
    for (let i = 1; i <= 4; i++) {
        board.channel(i as ChannelNum).setMotorType(MotorType.Motor310)
        basic.pause(500)
    }
    
    // Test each motor individually
    for (let i = 1; i <= 4; i++) {
        serial.writeLine("Testing M" + i + "...")
        
        // Set speeds (only one motor active)
        let speeds = [0, 0, 0, 0]
        speeds[i - 1] = 600
        board.setAllSpeeds(speeds[0], speeds[1], speeds[2], speeds[3])
        
        // Run for 2 seconds
        basic.pause(2000)
        
        // Read encoder
        let encoder = board.channel(i as ChannelNum).readRealtimeEncoder()
        serial.writeLine("M" + i + " encoder: " + encoder)
        
        // Stop
        board.stopAll()
        basic.pause(500)
    }
    
    serial.writeLine("All motors tested")
}

/**
 * Test 5: PWM control (for motors without encoder)
 */
function test_pwmControl(): void {
    serial.writeLine("=== Test 5: PWM Control ===")
    
    let board = getBoard()
    
    serial.writeLine("Running PWM ramp on M1...")
    let pwmValues = [0, 2000, 4000, 6000, 8000, 6000, 4000, 2000, 0]
    
    for (let pwm of pwmValues) {
        serial.writeLine("PWM: " + pwm)
        board.setAllPWM(pwm, 0, 0, 0)
        basic.pause(1000)
    }
    
    board.stopAll()
    serial.writeLine("PWM test complete")
}

/**
 * Test 6: Read all encoders simultaneously
 */
function test_readAllEncoders(): void {
    serial.writeLine("=== Test 6: Read All Encoders ===")
    
    let board = getBoard()
    
    // Run all motors at same speed
    serial.writeLine("Running all motors at 400...")
    board.setAllSpeeds(400, 400, 400, 400)
    
    // Read encoders every second for 5 seconds
    for (let i = 0; i < 5; i++) {
        basic.pause(1000)
        let encoders = board.readAllRealtimeEncoders()
        serial.writeLine("Encoders: M1=" + encoders[0] + " M2=" + encoders[1] + 
                        " M3=" + encoders[2] + " M4=" + encoders[3])
    }
    
    // Stop and read accumulated
    board.stopAll()
    basic.pause(500)
    
    let accumulated = board.readAllAccumulatedEncoders()
    serial.writeLine("Accumulated: M1=" + accumulated[0] + " M2=" + accumulated[1] + 
                    " M3=" + accumulated[2] + " M4=" + accumulated[3])
}

/**
 * Main test runner
 */
input.onButtonPressed(Button.A, function () {
    serial.writeLine("====================================")
    serial.writeLine("QuadEncoder Test Suite Starting...")
    serial.writeLine("====================================")
    basic.pause(1000)
    
    // Wait for I2C connection before running tests
    waitForI2CConnection()
    
    // Run tests sequentially
    test_initializeMotor()
    basic.pause(2000)
    
    test_speedRamp()
    basic.pause(2000)
    
    test_accumulatedEncoders()
    basic.pause(2000)
    
    test_allMotors()
    basic.pause(2000)
    
    test_pwmControl()
    basic.pause(2000)
    
    test_readAllEncoders()
    basic.pause(2000)
    
    serial.writeLine("====================================")
    serial.writeLine("All tests complete!")
    serial.writeLine("====================================")
})

input.onButtonPressed(Button.B, function () {
    serial.writeLine("Emergency stop - all motors off")
    getBoard().stopAll()
})

// Start checking I2C connection immediately on boot
serial.writeLine("====================================")
serial.writeLine("QuadEncoder Test Ready")
serial.writeLine("Checking I2C connection...")
serial.writeLine("====================================")

// Continuously check for I2C connection until found
waitForI2CConnection()

// Connection established - show ready state
basic.showIcon(IconNames.Heart)
serial.writeLine("")
serial.writeLine("Ready! Press A to start tests")
serial.writeLine("Press B for emergency stop")
