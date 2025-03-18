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

intake = SparkMax(16, SparkMax.MotorType.kBrushless)
shooter = SparkMax(17, SparkMax.MotorType.kBrushless)
