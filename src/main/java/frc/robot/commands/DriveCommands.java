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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
                    Logger.recordOutput("Drivetrain/Turn Speeds", speeds.omegaRadiansPerSecond);
                    drive.runVelocity(speeds);
                },
                drive);
    }

    public static Command joystickAdvancedDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
                    Logger.recordOutput("Drivetrain/Turn Speeds", speeds.omegaRadiansPerSecond);
                    drive.runAdvancedVelocity(speeds);
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), //* Math.abs(omega) * Math.abs(linearVelocity.getX()) * turnCorrection.get(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation());
                            drive.runVelocity(speeds);
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

        public static Command joystickDriveAt45(
                Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Pose2d> drivePose) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        final Rotation2d[] lockedRotation = { new Rotation2d() };

        return Commands.sequence(
                Commands.runOnce(() -> {
                Translation2d botTranslation = drivePose.get().getTranslation();

                List<Translation2d> hubs = List.of(
                        new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32)),
                        new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32))
                );
                Translation2d closestHub = botTranslation.nearest(hubs);

                double relX = botTranslation.getX() - closestHub.getX();
                double relY = botTranslation.getY() - closestHub.getY();

                double angleToBot = Math.atan2(relY, -relX);
                double snappedAngle = Math.floor(angleToBot / (Math.PI / 2.0)) * (Math.PI / 2.0) + (Math.PI / 4.0);

                lockedRotation[0] = new Rotation2d(snappedAngle);

                // Reset PID controller when command starts
                angleController.reset(drive.getRotation().getRadians());
                }),
                // Construct command
                Commands.run(
                        () -> {
                        // Get linear velocity
                        Translation2d linearVelocity =
                                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                        // Calculate angular speed
                        double omega = angleController.calculate(
                                drive.getRotation().getRadians(),
                                lockedRotation[0].getRadians());

                        // Convert to field relative speeds & send command
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                omega);

                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red;

                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation());
                        drive.runAdvancedVelocity(speeds);
                        },
                        drive)
        )
                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

       public static Command joystickDriveTagCentric(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Pose2d> drivePose) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);


        Logger.recordOutput("In Red Zone", drivePose.get().getX()  > (0));
        Logger.recordOutput("GetX", drivePose.get());


        // Construct command
        return Commands.run(
                        () -> {
                                if(drivePose.get().getX()  > (VisionConstants.aprilTagLayout.getFieldLength()-4.61) || drivePose.get().getX() < 4.61){
                                Translation2d botTranslation = drivePose.get().getTranslation();

                                List<Translation2d> hubs = List.of(
                                        new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32)),
                                        new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32))
                                );
                                Translation2d closestHub = botTranslation.nearest(hubs);

                                double relX = botTranslation.getX() - closestHub.getX();
                                double relY = botTranslation.getY() - closestHub.getY();

                                double angle = Math.atan(relY/relX);
                                if(DriverStation.getAlliance().get().equals(Alliance.Red))
                                        angle += Math.PI;


                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    angle);

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), //* Math.abs(omega) * Math.abs(linearVelocity.getX()) * turnCorrection.get(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation());
                            drive.runVelocity(speeds);
                                }else{
                                        double angle = Units.degreesToRadians(0);

                                // Get linear velocity
                                Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    angle);

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), //* Math.abs(omega) * Math.abs(linearVelocity.getX()) * turnCorrection.get(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation());
                            drive.runVelocity(speeds);

                                }
                        },
                        drive).beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));// Reset PID controller when command starts


    }

//     public static PathPlannerPath driveOverBump(Supplier<Pose2d> currentPose){
//         Translation2d botTranslation = currentPose.get().getTranslation();

//                                 List<Translation2d> hubs = List.of(
//                                         new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32)),
//                                         new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32))
//                                 );
//                                 Translation2d closestHub = botTranslation.nearest(hubs);

//                                 double relX = botTranslation.getX() - closestHub.getX();
//                                 double relY = botTranslation.getY() - closestHub.getY();

//                                 double angle = Math.atan(relY/relX);
//                                 if(DriverStation.getAlliance().get().equals(Alliance.Red))
//                                         angle += Math.PI;
//         Pose2d nextPose = currentPose.get().plus(new Transform2d(Units.inchesToMeters(47*Math.signum(relX)),Units.inchesToMeters(0),Rotation2d.fromRadians(angle).minus(currentPose.get().getRotation())));
//         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose.get(),nextPose);
//         List<PathPoint> pathPoints = new ArrayList<PathPoint>();
//         for(Waypoint point: waypoints){
//                 if(!(point.nextControl() == null))
//                         pathPoints.add(new PathPoint(point.nextControl()));
//         }
//         RobotConfig constraints = RobotContainer.drive.pathConfig;
//         return (pathPoints==null)?null:PathPlannerPath.fromPathPoints(pathPoints, new PathConstraints(0.5, 0.75, Math.PI/2, Math.PI/1.5), new GoalEndState(MetersPerSecond.of(0), nextPose.getRotation()));


//     }

    public static PathPlannerPath driveOverBump(Supplier<Pose2d> currentPose) {
    Pose2d robotPose = currentPose.get();
    Translation2d botTranslation = robotPose.getTranslation();

    // Define the hubs (center of the stages/bumps)
    List<Translation2d> hubs = List.of(
            new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32)),
            new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32))
    );
    Translation2d closestHub = botTranslation.nearest(hubs);

    // Calculate direction FROM hub TO bot
    double relX = botTranslation.getX() - closestHub.getX();
    double relY = botTranslation.getY() - closestHub.getY();

    // The angle from the hub to the robot
    double angleToBot = Math.atan2(relY, relX);

    // To drive "Over the bump", we want to move toward the hub and out the other side.
    // We create a translation vector pointing from the bot toward the hub,
    // then extend it so the robot ends up on the opposite side.
    double distanceOverBump = Units.inchesToMeters(47); // Adjust distance as needed

    // Direction vector toward the hub (inverted relX/relY)
        // Calculate the magnitude (distance from hub to bot)
        double magnitude = Math.hypot(relX, relY);

        // Create a unit vector pointing FROM the bot TOWARD the hub
        // We divide by magnitude to get a length of 1.0, then multiply by our desired distance
        Translation2d driveDirection = new Translation2d(-relX / magnitude, -relY / magnitude);

        // Calculate the final target point
        Translation2d targetTranslation = botTranslation.plus(
        driveDirection.times(distanceOverBump) // Move 60 inches along that vector
        );

    // Keep the robot facing the same way or face the movement direction
    Rotation2d targetRotation = robotPose.getRotation();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        robotPose,
        new Pose2d(10,6.02, targetRotation)
        );
        Logger.recordOutput("Bump Command/Wanted Pose", new Pose2d(targetTranslation, targetRotation));
        // 2. Convert Waypoints to PathPoints
        // We use the anchor point (the actual XY position) of each waypoint
        // List<PathPoint> pathPoints = waypoints.stream()
        // .map(waypoint -> new PathPoint(waypoint.anchor()))
        // .toList();

        // // 3. Return using fromPathPoints
        // PathPlannerPath finalPath = PathPlannerPath.fromPathPoints(
        // pathPoints,
        // new PathConstraints(2.0, 2.5, Units.degreesToRadians(360), Units.degreesToRadians(540)),
        // new GoalEndState(0.0, robotPose.getRotation())
        //);
        PathPlannerPath finalPath = new PathPlannerPath(waypoints, new PathConstraints(2.0, 2.5, Units.degreesToRadians(360), Units.degreesToRadians(540)),
        new IdealStartingState(0, currentPose.get().getRotation()),
        new GoalEndState(0.0, robotPose.getRotation()));
        finalPath.preventFlipping = false;
        Logger.recordOutput("Bump Path",  finalPath.getPathPoses().toArray(new Pose2d[finalPath.getPathPoses().size()]));
        return finalPath;
}

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius =
                                            (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
