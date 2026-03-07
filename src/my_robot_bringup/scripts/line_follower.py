#include "DeviceDriverSet_xxx0.h"
#include <HCSR04.h>

#DeviceDriverSet_ITR20001 AppITR20001

# Motor control pins
#define PIN_Motor_STBY 3
#define PIN_Motor_PWMA 5
#define PIN_Motor_AIN_1 7
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8

# Array to store the 3 read sensors
colours = [0,0,0]
# Speed for every movement except right turning 
speed = 60
# Counter to hold which angles we have searched so that the robot does not move random angles used in the switch case
searchStep = 0

# Calibrated black colour minimum and maximum
# These might be changed for the actual demo during preperation time
blackMax = 900
blackMin = 500

# ants that will be used throughout
realDelay = 150
# Ammount to move for angles
rightThirty = 500
leftThirty = 500
leftSixty = 1200
rightSixty = 1000
leftNinety = 1600
rightNinety = 1400
leftOneEightey = 3200
# rightOneEighty = 3000 # Not tested (not used)
# fullRot = realDelay*39 # Not tested (not used)

# --------------------------
# Motor Control Functions
# --------------------------
def rightFwd() :
    digitalWrite(PIN_Motor_STBY, HIGH)
    digitalWrite(PIN_Motor_AIN_1, HIGH)
    analogWrite(PIN_Motor_PWMA, speed)


def rightRev() :
    digitalWrite(PIN_Motor_STBY, HIGH)
    digitalWrite(PIN_Motor_AIN_1, LOW)
    analogWrite(PIN_Motor_PWMA, speed)


def leftFwd() :
    digitalWrite(PIN_Motor_STBY, HIGH)
    digitalWrite(PIN_Motor_BIN_1, HIGH)
    analogWrite(PIN_Motor_PWMB, speed)


def leftRev() :
    digitalWrite(PIN_Motor_STBY, HIGH)
    digitalWrite(PIN_Motor_BIN_1, LOW)
    analogWrite(PIN_Motor_PWMB, speed)


def stopMov() :
    digitalWrite(PIN_Motor_STBY, LOW)
    analogWrite(PIN_Motor_PWMA, 0)
    analogWrite(PIN_Motor_PWMB, 0)


# --------------------------
# Basic Turning Functions
# --------------------------

def turnRight( realDelay) :
    speed = 70
    rightRev()# right motor in reverse
    leftFwd() # left motor forward
    delay(realDelay)
    stopMov()
    speed = 60


def turnLeft( realDelay) :
    rightFwd()# right motor forward
    leftRev() # left motor in reverse
    delay(realDelay)
    stopMov()


def smartTurnRight( totalDuration,stepDelay = 100) :
    # Let the counter be equal to 0
    elapsed = 0
    # While the angle is less then the expected turn
    while (elapsed < totalDuration):
        # Continue turning right
        turnRight(stepDelay)

        # Read sensor again
        colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
        colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
        colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()

        # Check if there is a black line in one of the sensors if so return to continue executing
        if (blackMin < colours[1] < blackMax) or  (blackMin < colours[0] < blackMax) or (blackMin < colours[2] < blackMax) :
            Serial.println("Line found during right turn!")
            return True
        
        # Increment the steps
        elapsed += stepDelay
        
    return False


def smartTurnLeft( totalDuration,stepDelay = 100) :
    # Let the counter be equal to 0
    elapsed = 0
    # While the angle is less then the expected turn
    while (elapsed < totalDuration):
        # Continue turning left
        turnLeft(stepDelay)

        # Read sensor again
        colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
        colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
        colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()

        # Check if there is a black line in one of the sensors if so return to continue executing
        if (blackMin < colours[1] < blackMax) or (blackMin < colours[0] < blackMax) or (blackMin < colours[2] < blackMax):
            Serial.println("Line found during left turn!")
            return True
        
        # Increment the steps
        elapsed += stepDelay
    
    return False
    


def setup() :
    Serial.begin(9600)
    #AppITR20001.DeviceDriverSet_ITR20001_Init()


def loop() :
    Serial.print("ITR20001_getAnaloguexxx_L=")
    colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
    Serial.println(colours[0])
    Serial.print("ITR20001_getAnaloguexxx_M=")
    colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
    Serial.println(colours[1])
    Serial.print("ITR20001_getAnaloguexxx_R=")
    colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()
    Serial.println(colours[2])

    # Call the follow line function
    followLine()


def moveForwardWhileOnTrack() :
    # Move foward
    rightFwd()
    leftFwd()
    # Scan
    while (True):
        colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
        colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
        colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()

        # If the middle is not black
        if (not (blackMin < colours[1] < blackMax)) :
            # Stop moving and return
            stopMov()
            break
        
        delay(10) # micro-loop
    


def followLine():
    #colours[] -> 0:L, 1:M, 2:R
    if(blackMin < colours[1] < blackMax): #if M==black, move forward
        moveForwardWhileOnTrack()
    elif (blackMin < colours[0] < blackMax) :#if L==black, turn a bit to the left to correct
        turnLeft(realDelay)
    elif (blackMin < colours[2] < blackMax):#if R==black, turn a bit to the right to correct
        turnRight(realDelay)
    #if nowhere is black, try to find the path again.
    else:
        found = False
        if searchStep == 0:
            Serial.println("Search Step 0: Turn right 30")
            # Turn 30 right whilst searching 
            found = smartTurnRight(rightThirty)
        elif searchStep == 1: 
            Serial.println("Search Step 1: Turn left 60")
            # Turn 30 left to reset
            turnLeft(leftThirty)
            # Turn 60 left whilst searching 
            found = smartTurnLeft(leftSixty)
                
        elif searchStep == 2:
            Serial.println("Search Step 2: Turn right 90")
            # Turn 60 right to reset
            turnRight(rightSixty)
            # Turn 90 right whilst searching 
            found = smartTurnRight(rightNinety)
                
        elif searchStep == 3:
            Serial.println("Search Step 3: Turn left 90")
            # Turn 90 left to reset
            turnLeft(leftNinety)
            # Turn 90 left whilst searching 
            found = smartTurnLeft(leftNinety)
                
        elif searchStep == 4:
            Serial.println("Search Step 4: Turn left 180")
            # Turn 180 left whilst searching - only 90 left to turn 180
            found = smartTurnLeft(leftNinety)
                
        elif searchStep == 5:
            Serial.println("Search Step 5: Turn left 360")
            # Turn a whole turn whilst searching 
            found = smartTurnLeft(leftOneEightey)
                
        else:
            Serial.println("Search failed. Reversing...")
            # Reverse and restart the loop
            rightRev()
            leftRev()
            delay(realDelay)
            stopMov()
            searchStep = 0
                
            
        if (found) :
            # reset search if line found
            searchStep = 0
        else:
            # continue search if not found
            searchStep+=1