from controller import Robot, DistanceSensor, Motor
import random
# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

#4096 means that a big amount of light is measured (an obstacle is close) 
#0 means that no light is measured (no obstacle).
DETECT_DISTANCE = 80.0

def fight(ps, leftMotor, rightMotor):
    while robot.step(TIME_STEP) != -1:
        # read sensors outputs
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
        
        # detect obstacles
        right_obstacle = psValues[0] > DETECT_DISTANCE or psValues[1] > DETECT_DISTANCE or psValues[2] > DETECT_DISTANCE
        left_obstacle = psValues[5] > DETECT_DISTANCE or psValues[6] > DETECT_DISTANCE or psValues[7] > DETECT_DISTANCE
        
        if left_obstacle:
            # turn left for 2 steps
            leftSpeed  = -1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # go back for 2 steps
            robot.step(TIME_STEP)
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # ram for 2 steps
            robot.step(TIME_STEP)
            leftSpeed  = 1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        elif right_obstacle:
            # turn right for 2 steps
            leftSpeed  = 1 * MAX_SPEED
            rightSpeed = -1 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # go back for 2 steps
            robot.step(TIME_STEP)
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            # ram for 2 steps
            robot.step(TIME_STEP)
            leftSpeed  = 1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            
            robot.step(TIME_STEP)
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        
        # go forward for 5 steps
        number = random.randint(0, 6)
        if number==0:
       #turn three steps right
            for i in range(0, 3):
                robot.step(TIME_STEP)
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        elif number==1:
         #turn three steps left
            for i in range(0, 3):
                robot.step(TIME_STEP)
                leftSpeed  = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        else:
         #go five steps forward
            for i in range(0, 5):
                robot.step(TIME_STEP)
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        
# create the Robot instance.
robot = Robot()

# initialize devices
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

fight(ps, leftMotor, rightMotor)