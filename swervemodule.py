#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

from phoenix5.sensors import CANCoder

from rev import SparkMax

kWheelRadius = 0.0508
kEncoderResolution = 42
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau
wheelCircumference = 2 * math.pi * kWheelRadius
gearRatio = 6.75

class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        encoder: int
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        :param driveEncoderChannelA:   DIO input for the drive encoder channel A
        :param driveEncoderChannelB:   DIO input for the drive encoder channel B
        :param turningEncoderChannelA: DIO input for the turning encoder channel A
        :param turningEncoderChannelB: DIO input for the turning encoder channel B
        """

        self.driveMotor = SparkMax(driveMotorID, SparkMax.MotorType.kBrushless)
        self.turningMotor = SparkMax(turningMotorID, SparkMax.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()

        driveEncoderRPM = self.driveEncoder.getVelocity()
        driveRPM = driveEncoderRPM / gearRatio
        self.driveMPS = (driveRPM * wheelCircumference) / 60

        drivePosition = self.driveMotor.getEncoder().getPosition()
        self.driveDistance = (drivePosition / gearRatio) * wheelCircumference

        self.encoder = encoder

        self.turningEncoder = CANCoder(self.encoder)
        self.angleRadians = math.radians(self.turningEncoder.getAbsolutePosition())

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(0, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)


    def update(self) -> None:
        """Updates the internal module state from sensors."""
        self.driveEncoder = self.driveMotor.getEncoder()

        driveEncoderRPM = self.driveEncoder.getVelocity()
        driveRPM = driveEncoderRPM / gearRatio
        self.driveMPS = (driveRPM * wheelCircumference) / 60

        drivePosition = self.driveMotor.getEncoder().getPosition()
        self.driveDistance = (drivePosition / gearRatio) * wheelCircumference

        self.turningEncoder = CANCoder(self.encoder)
        self.angleRadians = math.radians(self.turningEncoder.getAbsolutePosition())

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveMPS,
            wpimath.geometry.Rotation2d(self.angleRadians),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveDistance,
            wpimath.geometry.Rotation2d(self.angleRadians),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.angleRadians)

        # Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        desiredState.cosineScale(encoderRotation)

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveMPS, desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.angleRadians, desiredState.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)