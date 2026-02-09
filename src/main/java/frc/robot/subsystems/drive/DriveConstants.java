// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.module_average_kA_Drive;
import static frc.robot.subsystems.drive.DriveConstants.module_average_kA_Turn;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.3; // 5.3; // 4.8;
    public static final double odometryFrequency = 250.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(25);
    public static final double wheelBase = Units.inchesToMeters(25);
    public static final double driveBaseRadius = Math.sqrt((Math.pow(trackWidth, 2) + Math.pow(wheelBase, 2)))
            / 2.0; // Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    // kevin pfeffer was here
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation =
            Rotation2d.fromRotations(0.331787); // -0.143311); // new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRotations(-0.313965); // 0.204834);
    public static final Rotation2d backLeftZeroRotation =
            Rotation2d.fromRotations(0.478760); // 0.464355); // 0.453857);
    public static final Rotation2d backRightZeroRotation = Rotation2d.fromRotations(0.176025); // 0.171875);

    // Device CAN IDs
    public static final int pigeonCanId = 0;

    public static final int frontLeftDriveCanId = 2;
    public static final int backLeftDriveCanId = 8;
    public static final int frontRightDriveCanId = 4;
    public static final int backRightDriveCanId = 6;

    public static final int frontLeftTurnCanId = 1;
    public static final int backLeftTurnCanId = 7;
    public static final int frontRightTurnCanId = 3;
    public static final int backRightTurnCanId = 5;

    public static final int frontLeftCancoderId = 1;
    public static final int frontRightCancoderId = 2;
    public static final int backLeftCancoderId = 4;
    public static final int backRightCancoderId = 3;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 40;
    public static final double wheelRadiusMeters = 0.051; // In Meters     //Units.inchesToMeters(2);
    public static final double driveMotorReduction =
            6.75; //     (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.008; // 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.21962; // 0.19416; // 0; // 0.17266; // 0.0;
    public static final double driveKv = 0.13561; // 0.13421; // 0; // 0.14037; // .135; // .12; // .15; // 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 150.0 / 7; // 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

    // kA Constants
    public static final double module_average_kA_Drive = 0.3;
    public static final double module_average_kA_Turn = 0.574665;
    // Turn PID configuration
    public static final double turnKp = 4.0; // 2.0; // 1.6; // 0.8; // 0.575; // 0.01; // 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnKs = 2.44234;
    public static final double turnKv = 0.67128;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
    public static final double angularKa = 0;
    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI =
            robotMassKg * (trackWidth / 2) * (module_average_kA_Turn / module_average_kA_Drive); // 6.883; // 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF));

    public static final double skidThresholdX = 0.035;//0.03;
    public static final double skidThresholdY = 0.035;//0.029;//0.029

    public static final double baseXDriveSTDEV = 0.09;
    public static final double baseYDriveSTDEV = 0.025;
    public static final double baseThetaDriveSTDEV = Units.degreesToRadians(0.001);

    public static final double baseXVisionSTDEV = 0.1;
    public static final double baseYVisionSTDEV = 0.1;
    public static final double baseThetaVisionSTDEV = 999999;

    //public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.)

    // Module 1 Drive kA = 0.016577;
    // Module 2 Drive kA = 0.019138;
    // Module 3 Drive kA = 0.012256;
    // Module 4 Drive kA = 0.022477;

    // Module 1 Turn kA = 0.18754;
    // Module 2 Turn kA =0.96179;
    // Module 3 Turn kA = 0.054505;
    // Module 4 Turn kA = 0.3;

}
