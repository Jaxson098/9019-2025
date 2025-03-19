#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule
from navx import AHRS
from networktables import NetworkTables

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.23495, 0.23495)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.23495, -0.23495)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.23495,0.234951)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.23495, -0.23495)

        self.frontLeft = swervemodule.SwerveModule(5, 6, 13, False)
        self.frontRight = swervemodule.SwerveModule(7, 8, 10, False)
        self.backRight = swervemodule.SwerveModule(4, 2, 12, False)
        self.backLeft = swervemodule.SwerveModule(3, 11, 9, False)

        self.gyro = AHRS.create_spi()

        NetworkTables.initialize(server='10.90.19.2')

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.frontLeft.update()
        self.frontRight.update()
        self.backLeft.update()
        self.backRight.update()

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )

        self.frontLeft.update()
        self.frontRight.update()
        self.backLeft.update()
        self.backRight.update()

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

        self.table = NetworkTables.getTable('table')

        self.table.putNumber("xSpeed", xSpeed)
        self.table.putNumber("ySpeed", ySpeed)
        self.table.putNumber("rot", rot)

        self.table.putNumber("frontLeft desired angle:", math.degrees(swerveModuleStates[0].angle.radians()))
        self.table.putNumber("frontRight desired angle:", math.degrees(swerveModuleStates[1].angle.radians()))
        self.table.putNumber("backLeft desired angle:", math.degrees(swerveModuleStates[2].angle.radians()))
        self.table.putNumber("backRight desired angle:", math.degrees(swerveModuleStates[3].angle.radians()))
    
        self.table.putNumber("frontLeft angleRadians", self.frontLeft.angleRadians)
        self.table.putNumber("frontLeft driveMPS", self.frontLeft.driveMPS)
        self.table.putNumber("frontLeft driveDistance", self.frontLeft.driveDistance)

        self.table.putNumber("frontRight angleRadians", self.frontRight.angleRadians)
        self.table.putNumber("frontRight driveMPS", self.frontRight.driveMPS)
        self.table.putNumber("frontRight driveDistance", self.frontRight.driveDistance)

        self.table.putNumber("backRight angleRadians", self.backRight.angleRadians)
        self.table.putNumber("backRight driveMPS", self.backRight.driveMPS)
        self.table.putNumber("backRight driveDistance", self.backRight.driveDistance)

        self.table.putNumber("backLeft angleRadians", self.backLeft.angleRadians)
        self.table.putNumber("backLeft driveMPS", self.backLeft.driveMPS)
        self.table.putNumber("backLeft driveDistance", self.backLeft.driveDistance)
        self.table.putNumber("gyro.getRotation2d.degrees()", self.gyro.getRotation2d().degrees())

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )