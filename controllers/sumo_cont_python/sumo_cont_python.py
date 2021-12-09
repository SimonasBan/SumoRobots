from controller import Robot, DistanceSensor, Motor
import random

# Time in [ms] of a simulation step
TIME_STEP = 64

# Maximum speed constant
MAX_SPEED = 6.28

# Detection distance constant
# A value of 4096 means that a big amount of light is measured (an obstacle is close) 
# A value of 0 means that no light is measured (no obstacle)
DETECT_DISTANCE = 80.0

# Robot fighting behavior function
def fight(ps, leftMotor, rightMotor):
    # Feedback loop: step simulation until receiving an exit event
    while robot.step(TIME_STEP) != -1:
        # Read sensors outputs
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
        
        # Detect obstacles
        right_obstacle = psValues[0] > DETECT_DISTANCE or psValues[1] > DETECT_DISTANCE or psValues[2] > DETECT_DISTANCE
        left_obstacle = psValues[5] > DETECT_DISTANCE or psValues[6] > DETECT_DISTANCE or psValues[7] > DETECT_DISTANCE
        
        # If the obstacle is to the left
        if left_obstacle:
            # Set speed to turn left
            leftSpeed  = -1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
            # Turn left
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            # Advance time and turn left
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # Advance time
            robot.step(TIME_STEP)
            # Set speed to go backwards
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            # Go backwards
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            # Advance time and go backwards
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # Advance time
            robot.step(TIME_STEP)
            # Set speed to go forward
            leftSpeed  = 1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
            # Go forward
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            # Advance time and go forward
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        # If the obstacle is to the right
        elif right_obstacle:
            # Set speed to turn right
            leftSpeed  = 1 * MAX_SPEED
            rightSpeed = -1 * MAX_SPEED
            # Turn right
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            # Advance time and turn right
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # Advance time
            robot.step(TIME_STEP)
            # Set speed to go backwards
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            # Go backwards
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            # Advance time and go backwards
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # Advance time
            robot.step(TIME_STEP)
            # Set speed to go forward
            leftSpeed  = 1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
            # Go forward
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # Advance time and go forward
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        
        # Random number generator to choose which direction to move
        number = random.randint(0, 6)
        if number==0:
            # Turn right for three steps
            for i in range(0, 3):
                robot.step(TIME_STEP)
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        elif number==1:
            # Turn left for three steps
            for i in range(0, 3):
                robot.step(TIME_STEP)
                leftSpeed  = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        else:
            # Go forward for five steps
            for i in range(0, 5):
                robot.step(TIME_STEP)
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        
# Create the Robot instance
robot = Robot()

# Initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Begin fighting behavior
fight(ps, leftMotor, rightMotor)