import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

from phoenix5.sensors import CANCoder

from rev import SparkMax

liftMotor1 = SparkMax(14, SparkMax.MotorType.kBrushless)
liftMotor2 = SparkMax(15, SparkMax.MotorType.kBrushless)

intakeMotor = SparkMax(16, SparkMax.MotorType.kBrushless)
angleMotor = SparkMax(17, SparkMax.MotorType.kBrushless)

CheckBol = False

motorState = 0

def useLift(rightBump, leftBump, leftY, rightY):

    if rightBump:
        intakeMotor.setVoltage(9)
    elif leftBump:
        intakeMotor.setVoltage(-9)
    else:
        intakeMotor.setVoltage(0)

    intakeMotor.setVoltage(leftY)
    angleMotor.setVoltage(rightY)