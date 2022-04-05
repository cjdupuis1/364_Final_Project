# Put the various CircuitPython import statements here
import board
import digitalio
import busio
import time
import adafruit_bno055
import pwmio
import pulseio
# Configuration of the simulator (will have to remove these lines
# when running on real hardware) 
import dynamic_model
dynamic_model.enable_wind=True   # Set this to True to add a moment from the wind

# ***NOTE*** Do not get the various files which are part of the simulator
# (adafruit_bno055.py, busio.py, board.py, etc.) confused with similarly
# named files used for actual CircuitPython hardware. Do NOT copy the
# simulator versions onto your Feather microcontroller! 
frequency = 40
duty_max = 65535
period = 25
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

forward = pulseio.PulseIn(board.D6, 1, False)

# forward = pulseio.PulseIn(board.D6) #use this when fwd command is inputed

rotation = pulseio.PulseIn(board.D5)
left_fan = pwmio.PWMOut(board.D9,1.5/period * duty_max ,frequency)
right_fan = pwmio.PWMOut(board.D10, 1.5/period * duty_max, frequency)
rot_fan = pwmio.PWMOut(board.D11, 1.5/period * duty_max, frequency)

# Now create objects for the I2C bus, for the BNO055,
# PulseIn, PWMOut, and/or DigitalInOut
#
# Will only work with pins used as follows (per pin
# definitions in board.py)
#    SCL: SCL
#    SDA: SDA
#    D5: Rotational command pulse input from RC receiver
#    D6: Forward command pulse input from RC receiver (fixed at 1500us)
#    D9: PWM pulse out to left-side fan.
#        Running this fan forwards helps turn right,
#        Running backwards helps turn left 
#    D10: PWM pulse out to right-side fan
#        Running this fan forwards helps turn left
#        Running backwards helps turn right
#    D13: LED output. Use to indicate calibrationstate of BNO055
#
# You may also want variables for other values such as your chosen PWM frequency,


# When adding integration capability
# You will need a variable to accumulate the integral term, etc. 
# You will also need to keep track of the time (from time.monotonic())
# of the previous iteration from the loop so you can accumulate the
# right amount
prev_time = time.monotonic()
i_term = 0
# Start an infinite loop here:
while True:
    while len(rotation) == 0:
        pass
    rotation.pause()
    rot_pulse = rotation.popleft() / 1000
    print("Pulse:", rot_pulse)
    rotation.clear()
    rotation.resume()
    # Wait until pulses have been received on pin D5

    # Then get them with the popleft() method
    # keeping the newest plausible pulse. Then pause, clear, and resume. 
    if(sensor.calibrated):
        led.value = True
    else:
        led.value = False
        
    # Check whether the BNO055 is calibrated
    # and turn on the LED on D13 as appopriate

    # Extract the euler angles (Heading, Roll, Pitch)
    # in degrees from the IMU
    eulers = sensor.euler
    # Determine the commanded orientation based on from your pulse input from
    # pin D5
    head_d = (rot_pulse - 1) * 360 - 180
    print("Desired:" ,head_d)
    print("eulers:", eulers)
    error = ((head_d - eulers[0] + 360 + 180)%360 - 180)
    print("error:", error)
    k_p = 0.005
    p_term = k_p * error
    k_d = 0.075
    d_term = k_d * sensor.gyro[2]
    k_i = 0.0005
    current_time = time.monotonic()
    dt = current_time - prev_time
    i_term = i_term + k_i * error * dt
    prev_time = current_time
    if i_term > 0.1:
        i_term = 0.1
    elif i_term < -0.1:
        i_term = -0.1
    print("dt:", dt)
    print("i_term:", i_term)
    # Select a coefficient for the proportional term
    # It will probably have units similar to output_command_ms/degree
    # Determine the proportional term by obtaining the error (subtracting
    # the commanded and actual headings), then multiplying by the proportional
    # coefficient
    #
    # TIP: If you just do simple subtraction you have a problem if the
    # commanded heading is (for example) +179 deg and the actual heading is
    # -179 deg. as it will try to turn the long way around.
    # The simplest solution is to add 360+180 deg. to the error, then use the
    # Python modulus operator (%) to get the remainder when dividing by
    # 360, then subtract 180 deg., e.g. replace simple subtraction with
    #  ((Heading_command - Heading + 360 + 180) % 360  - 180)
    output_ms = p_term + d_term + i_term
    if output_ms > 0.5:
        output_ms = 0.5
    elif output_ms < -0.5:
        output_ms = -0.5
        
    print("output ms:", output_ms)
    # To start with use just the proportional term to determine the output rotation
    # command, which is an offset in ms from the nominal 1.5 ms that commands
    # the fully reversing motors to not move.
    right_ms = 1.5 - output_ms
    left_ms = 1.5 + output_ms
    print("Left:", left_ms)
    print("right:", right_ms)
    if right_ms == 1.5:
        print("going straight")
    elif right_ms > 1.5:
        print("going left")
    else:
        print("going right")
    left_fan.duty_cyc = left_ms/period * duty_max
    right_fan.duty_cyc = right_ms/period * duty_max
    print("left duty:", left_fan.duty_cyc, " right duty:", right_fan.duty_cyc)
    # Bound the output rotation command so that it cannot exceed 0.5 or be less than
    # -0.5
    # Apply the output rotation command in opposite senses to determine the duty
    # cycle for the PWM outputs to the left- and right- side fans (pins D7, D8)

    pass # Done with loop


# Once you have the above working, you can test it out (making sure it is saved
# as "main.py" by running it with the simulator "python simulator.py"

# You can control the commanded heading by clicking in the lower-right quadrant.
# The x-y arrow in the upper-left quadrant should show you the orientation of
# your craft. You can add debugging prints as needed. Start out with a small
# proportional coefficient and increase it so that it makes the craft rotate
# at a reasonable rate. 

# You will find that the craft is extremely under-damped. This is because there is
# little air resistance to rotation.

# To get it to rapidly stabilize at the desired orientation you will need to add
# a derivative term. You can get the z axis rotation rate from the .gyro[2]
# property of the BNO055.
#
# Then you will need to define a derivative coefficient (which may have to be
# negative) represented as command_ms / (rad/sec). You can create the derivative
# by multiplying your derivative coefficient by the gyro rate and include it in the
# output rotation command.

# Once you have that working consider the case when some breeze tends to
# make the craft spontaneously rotate (set dynamic_model.enable_wind=True,
# above). To still equilibriate to the correct orientation you will need
# to add an integral term.
#
# Each time through the loop the integral gets added to it
# the integral coefficient * error * dt
# where dt is the difference in time (as measured with time.monotonic())
# from the previous loop iteration to this loop iteration. 
#  * Remember to use the TIP above when calculating the error.
#
# Every time you update the integral term you also have to bound it,
# lest it get too big and cause instability. This limit would probably
# have units of command_ms and would be a reasonably small fraction of
# the maximum command of 0.5 ms. Remember to bound it on both
# positive and negative sides. 
