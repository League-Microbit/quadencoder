import smbus
import struct
import time

UPLOAD_DATA = 1  # 1: total encoder data, 2: real-time encoder
                 #1: Receive total encoder data 2: Receive real-time encoder

MOTOR_TYPE = 1  # 1: 520 motor, 2: 310 motor, 3: TT motor with encoder disk, 4: TT DC geared motor, 5: L-type 520 motor
                #1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

# Create I2C communication object
bus = smbus.SMBus(1)  # 1 represents I2C bus number; modify based on your driver board's I2C bus
                       #1 represents the I2C bus number. You may need to modify it according to the I2C bus where your driver board is located.

# I2C Address
MOTOR_MODEL_ADDR = 0x26

# I2C Register Definition
MOTOR_TYPE_REG = 0x01
MOTOR_DEADZONE_REG = 0x02
MOTOR_PLUSELINE_REG = 0x03
MOTOR_PLUSEPHASE_REG = 0x04
WHEEL_DIA_REG = 0x05
SPEED_CONTROL_REG = 0x06
PWM_CONTROL_REG = 0x07

READ_TEN_M1_ENCODER_REG = 0x10
READ_TEN_M2_ENCODER_REG = 0x11
READ_TEN_M3_ENCODER_REG = 0x12
READ_TEN_M4_ENCODER_REG = 0x13

READ_ALLHIGH_M1_REG = 0x20
READ_ALLLOW_M1_REG = 0x21
READ_ALLHIGH_M2_REG = 0x22
READ_ALLLOW_M2_REG = 0x23
READ_ALLHIGH_M3_REG = 0x24
READ_ALLLOW_M3_REG = 0x25
READ_ALLHIGH_M4_REG = 0x26
READ_ALLLOW_M4_REG = 0x27

# Global variables
encoder_offset = [0] * 4
encoder_now = [0] * 4

# Write data to I2C register
def i2c_write(addr, reg, data):
    bus.write_i2c_block_data(addr, reg, data)

# Read data from I2C registers
def i2c_read(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

# Floating point number to byte
def float_to_bytes(f):
    return struct.pack('<f', f) # Little-endian

# Configure motor type
def set_motor_type(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_TYPE_REG, [data])

# Configure dead zone
def set_motor_deadzone(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_DEADZONE_REG, buf)

# Configure magnetic loop
def set_pluse_line(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSELINE_REG, buf)

# Configure the reduction ratio
def set_pluse_phase(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSEPHASE_REG, buf)

# Configure wheel diameter
def set_wheel_dis(data):
    bytes_data = float_to_bytes(data)
    i2c_write(MOTOR_MODEL_ADDR, WHEEL_DIA_REG, list(bytes_data))

# Control speed
def control_speed(m1, m2, m3, m4):
    speeds = [
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m4 >> 8) & 0xFF, m4 & 0xFF
    ]
    i2c_write(MOTOR_MODEL_ADDR, SPEED_CONTROL_REG, speeds)

# Control PWM (for motors without encoder)
def control_pwm(m1, m2, m3, m4):
    pwms = [
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m4 >> 8) & 0xFF, m4 & 0xFF
    ]
    i2c_write(MOTOR_MODEL_ADDR, PWM_CONTROL_REG, pwms)

# Read encoder data
def read_10_encoder():
    global encoder_offset
    formatted_values = []
    for i in range(4):
        reg = READ_TEN_M1_ENCODER_REG + i
        buf = i2c_read(MOTOR_MODEL_ADDR, reg, 2)
        encoder_offset[i] = (buf[0] << 8) | buf[1]
        if encoder_offset[i] & 0x8000:  # Check if the highest bit (sign bit) is 1
            encoder_offset[i] -= 0x10000 # Turn it into a negative number
        formatted_values.append("M{}:{}".format(i + 1, encoder_offset[i]))
    return ", ".join(formatted_values)

def read_all_encoder():
    global encoder_now
    formatted_values = []
    for i in range(4):
        high_reg = READ_ALLHIGH_M1_REG + (i * 2)
        low_reg = READ_ALLLOW_M1_REG + (i * 2)
        high_buf = i2c_read(MOTOR_MODEL_ADDR, high_reg, 2)
        low_buf = i2c_read(MOTOR_MODEL_ADDR, low_reg, 2)
        
        high_val = high_buf[0] <<8 | high_buf[1]
        low_val = low_buf[0] <<8 | low_buf[1]
        
        encoder_val = (high_val << 16) | low_val
        
        # Handle sign extension, assuming 32-bit signed integers
        if encoder_val >= 0x80000000:  # If greater than 2^31, it should be a negative number
            encoder_val -= 0x100000000  # Turn it into a negative number
        encoder_now[i] = encoder_val
        formatted_values.append("M{}:{}".format(i + 1, encoder_now[i]))
    return ", ".join(formatted_values) 

## Configure the following parameters according to your motor; set once and the driver board persists across power cycles
##The following parameters can be configured according to the actual motor you use. You only need to configure it once. The motor driver board has a power-off saving function.
def set_motor_parameter():

    if MOTOR_TYPE == 1:
        set_motor_type(1)  # Configure motor type
        time.sleep(0.1)
        set_pluse_phase(30)  # Configure the reduction ratio (see motor manual)
        time.sleep(0.1)
        set_pluse_line(11)  # Configure magnetic ring line (see motor manual)
        time.sleep(0.1)
        set_wheel_dis(67.00)  # Configure wheel diameter (measured)
        time.sleep(0.1)
        set_motor_deadzone(1600)  # Configure motor dead zone (empirically determined)
        time.sleep(0.1)

    elif MOTOR_TYPE == 2:
        set_motor_type(2)
        time.sleep(0.1)
        set_pluse_phase(20)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(48.00)
        time.sleep(0.1)
        set_motor_deadzone(1200)
        time.sleep(0.1)

    elif MOTOR_TYPE == 3:
        set_motor_type(3)
        time.sleep(0.1)
        set_pluse_phase(45)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(68.00)
        time.sleep(0.1)
        set_motor_deadzone(1250)
        time.sleep(0.1)

    elif MOTOR_TYPE == 4:
        set_motor_type(4)
        time.sleep(0.1)
        set_pluse_phase(48)
        time.sleep(0.1)
        set_motor_deadzone(1000)
        time.sleep(0.1)

    elif MOTOR_TYPE == 5:
        set_motor_type(1)
        time.sleep(0.1)
        set_pluse_phase(40)
        time.sleep(0.1)
        set_pluse_line(11)
        time.sleep(0.1)
        set_wheel_dis(67.00)
        time.sleep(0.1)
        set_motor_deadzone(1600)
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        t = 0
        print("please wait...")
        set_motor_parameter() # Set your own motor parameters

        while True:
            t += 10
            M1 = t
            M2 = t
            M3 = t
            M4 = t

            if MOTOR_TYPE == 4:
                control_pwm(M1*2, M2*2, M3*2, M4*2)
            else:
                control_speed(M1, M2, M3, M4)  # Send commands directly to control the motor
            
            if t> 1000 or t < -1000:
                t = 0

            if UPLOAD_DATA == 1:
                now_string = read_all_encoder()  # Read the accumulated encoder data
                print(now_string)
            elif UPLOAD_DATA == 2:
                offset_string = read_10_encoder()  # Read real-time encoder data
                print(offset_string)          
            time.sleep(0.1)

    except KeyboardInterrupt:
            control_pwm(0, 0, 0, 0)  # Stop the motor
