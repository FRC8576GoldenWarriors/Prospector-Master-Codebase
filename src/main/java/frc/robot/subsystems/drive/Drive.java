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
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.poseEstimation.BumpDetector;
import frc.robot.util.poseEstimation.CollisionDetector;
import frc.robot.util.poseEstimation.EnhancedSwervePoseEstimator;
import frc.robot.util.LocalADStarAK;
import java.util.Arrays;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.stream.DoubleStream;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final BumpDetector bumpDetector;
    private final CollisionDetector collisionDetector;
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    private RobotConfig pathConfig;
    private CANrange range;
    private CANrange range2;

    private final double[] skidAmountX = new double[4];
    private final double[] skidAmountY = new double[4];

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    // Access to this is guarded by `odometryLock` in most places; mark volatile to
    // ensure visibility for any reads done without the lock.
    private volatile Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final EnhancedSwervePoseEstimator poseEstimator = new EnhancedSwervePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(DriveConstants.baseXDriveSTDEV, DriveConstants.baseYDriveSTDEV, DriveConstants.baseThetaDriveSTDEV),
            VecBuilder.fill(DriveConstants.baseXVisionSTDEV, DriveConstants.baseYVisionSTDEV, DriveConstants.baseThetaVisionSTDEV));
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        bumpDetector = new BumpDetector(gyroIO.getPitchStatusSignal(), gyroIO.getRollStatusSignal(), Hertz.of(100));
        collisionDetector = new CollisionDetector(gyroIO.getXAccelerationStatusSignal(), gyroIO.getYAccelerationStatusSignal(), Hertz.of(100));

        try{
            pathConfig = RobotConfig.fromGUISettings();
        }catch(Exception e){
            e.printStackTrace();
        }
        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(7.5, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                this.pathConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
                range = new CANrange(1);
                range2 = new CANrange(2);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            boolean[] isSkidding = this.calculateSkidding();
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                if (!isSkidding[moduleIndex]) {
                    lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
                } else {
                    modulePositions[moduleIndex] = lastModulePositions[moduleIndex];
                }
            }

            double xDeviation = DriveConstants.baseXDriveSTDEV;
            double yDeviation = DriveConstants.baseYDriveSTDEV;
            double thetaDeviation = DriveConstants.baseThetaDriveSTDEV;

            // Skid: reuse the earlier computed isSkidding array
            boolean anySkidding = false;
            for (boolean bool : isSkidding) {
                if (bool) {
                    anySkidding = true;
                    break;
                }
            }

            if (anySkidding) {
                // If any of the modules are skidding calculate the mean value of their skid velocity
                double averageXSkid = DoubleStream.of(this.skidAmountX).average().orElse(0.0);
                double averageYSkid = DoubleStream.of(this.skidAmountY).average().orElse(0.0);

                Logger.recordOutput("averageXSkid", averageXSkid);
                Logger.recordOutput("averageYSkid", averageYSkid);

                xDeviation += Math.sqrt(averageXSkid/3);
                yDeviation += Math.sqrt(averageYSkid/3);
            }

            // Bump

            Pair<Double, Double> bumpStandardDeviations = bumpDetector.getBumpSTDDevs();

            xDeviation += bumpStandardDeviations.getFirst();
            yDeviation += bumpStandardDeviations.getSecond();

            // TODO Collision
            boolean anyCollision = collisionDetector.isColliding();

            // Deviations Update
            poseEstimator.setStateStdDevs(VecBuilder.fill(xDeviation, yDeviation,
                thetaDeviation));

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            boolean anyBumping = bumpDetector.isBumping();

            // Apply update
            if(!anyBumping)
                poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
        Logger.recordOutput("Drive/CanRange", getDetected());
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Command driveToPose(Pose2d wantedPose){
        Pathfinding.setStartPosition(getPose().getTranslation());
        return new DeferredCommand(() ->AutoBuilder.pathfindToPose(wantedPose, new PathConstraints(2.5, 3, 2, 3)),Set.of(this));
    }


    /** Resets the current odometry pose. */
    public void resetOdometry(Pose2d pose) {
        // Acquire odometry lock to avoid races with periodic odometry updates.
        odometryLock.lock();
        try {
            resetSimulationPoseCallBack.accept(pose);
            poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        } finally {
            odometryLock.unlock();
        }
    }

    public void resetGyro(Pose2d pose) {
        odometryLock.lock();
        try {
            resetSimulationPoseCallBack.accept(pose);
            gyroIO.resetHeading(pose.getRotation().getDegrees());
            poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        } finally {
            odometryLock.unlock();
        }
    }

    @AutoLogOutput(key = "Drive/CANRange Detected")
    public boolean getDetected(){
        return range.getIsDetected().getValue();
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        // Protect estimator from concurrent updates
        odometryLock.lock();
        try {
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            odometryLock.unlock();
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxSpeedMetersPerSec / driveBaseRadius;
    }

    public double getVelocity(){
        return gyroInputs.xVelocityRadPerSec+gyroInputs.yVelocityRadPerSec+(gyroInputs.yawVelocityRadPerSec);//(gyroInputs.zVelocityRadPerSec/1.5);
    }

    public boolean[] calculateSkidding() {
        SwerveModuleState[] moduleStates = getModuleStates();
        ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
        // Step 1: Create a measured ChassisSpeeds object with solely the rotation
        // component
        ChassisSpeeds rotationOnlySpeeds =
                new ChassisSpeeds(0.0, 0.0, currentChassisSpeeds.omegaRadiansPerSecond + .05);
        double[] xComponentList = new double[4];
        double[] yComponentList = new double[4];
        // Step 2: Convert it into module states with kinematics
        SwerveModuleState[] rotationalStates = kinematics.toSwerveModuleStates(rotationOnlySpeeds);
        // Step 3: Subtract the rotational states from the module states to get the
        // translational vectors and calculate the magnitudes.
        // These should all be the same direction and magnitude if there is no skid.
        for (int i = 0; i < moduleStates.length; i++) {
            double deltaX = moduleStates[i].speedMetersPerSecond * Math.cos(moduleStates[i].angle.getRadians())
                    - rotationalStates[i].speedMetersPerSecond * Math.cos(rotationalStates[i].angle.getRadians());
            double deltaY = moduleStates[i].speedMetersPerSecond * Math.sin(moduleStates[i].angle.getRadians())
                    - rotationalStates[i].speedMetersPerSecond * Math.sin(rotationalStates[i].angle.getRadians());
            xComponentList[i] = deltaX;
            yComponentList[i] = deltaY;
        }
        // Step 4: Compare all of the translation vectors. If they aren't the same, skid
        // is present.
        Arrays.sort(xComponentList);
        Arrays.sort(yComponentList);
        double deltaMedianX = (xComponentList[1] + xComponentList[2] + xComponentList[3] + xComponentList[0]) / 4;
        double deltaMedianY = (yComponentList[1] + yComponentList[2] + yComponentList[3] + yComponentList[0]) / 4;
        boolean[] areModulesSkidding = new boolean[4];

        for (int i = 0; i < 4; i++) {
            double deltaX = xComponentList[i];
            double deltaY = yComponentList[i];
            if (Math.abs(deltaX - deltaMedianX) > skidThresholdX
                    || Math.abs(deltaY - deltaMedianY) > skidThresholdY) { // 0.5 is the skid threshold in m/s
                areModulesSkidding[i] = true;
            } else {
                areModulesSkidding[i] = false;
            }
            skidAmountX[i] = Math.abs(deltaX - deltaMedianX);
            skidAmountY[i] = Math.abs(deltaY - deltaMedianY);
        }
        Logger.recordOutput("Drive/skidAmountX", skidAmountX);
        Logger.recordOutput("Drive/skidAmountY", skidAmountY);
        Logger.recordOutput("Drive/Skids", areModulesSkidding);
        return areModulesSkidding;
    }
}
