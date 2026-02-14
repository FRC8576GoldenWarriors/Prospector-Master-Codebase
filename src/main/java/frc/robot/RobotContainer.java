// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveX;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public static Drive drive;
    private final Vision vision;
    private final GyroIO gyro;
    //private final Shooter shooter;
    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    public static final CommandXboxController controller = new CommandXboxController(0);

    public Trigger resetHeadingTrigger = new Trigger(() -> controller.start().getAsBoolean());
    // Dashboard inputs
    private final Autos autos;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                gyro = new GyroIOPigeon2();
                drive = new Drive(
                        gyro,
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                        (pose) -> {});

                this.vision = new Vision(
                drive,
                drive::getChassisSpeeds,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera3Name, drive::getRotation));
                //shooter = new Shooter(0);
                autos = new Autos(drive);
                //intake = new Intake(new IntakeKrakenIO());
                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIOSim(driveSimulation.getGyroSimulation());
                drive = new Drive(
                        gyro,
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                drive,
                drive::getChassisSpeeds,
                new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                //shooter = null;
                autos = new Autos(drive);
                break;
            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                drive = new Drive(
                        gyro,
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, drive::getChassisSpeeds, new VisionIO() {}, new VisionIO() {});
                autos = null;
               // shooter = null;
                break;
        }

        // Set up auto routines
        // autos = new Autos(drive);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
               drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        // controller
        //         .a()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new Rotation2d()));
        //controller.a().onTrue(new DriveX(drive, -0.5).until(()->drive.getDetected()));
        //controller.a().onTrue(intake.setWantedState(IntakeStates.Rest));
        // Switch to X pattern       when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        //controller.x().onTrue(intake.setWantedState(IntakeStates.Intake));
        controller.x().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> Rotation2d.fromDegrees(45)));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Example Coral Placement Code
        // TODO: delete these code for your own project
        if (Constants.currentMode == Constants.Mode.SIM) {
            // L4 placement
            controller.y().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                            new Translation2d(0.4, 0),
                            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                            Meters.of(2),
                            MetersPerSecond.of(1.5),
                            Degrees.of(-80)))));
            // L3 placement
            controller.b().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                            new Translation2d(0.4, 0),
                            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                            Meters.of(1.35),
                            MetersPerSecond.of(1.5),
                            Degrees.of(-60)))));
        }
        resetHeadingTrigger.onTrue(new InstantCommand(() -> {
            Pose2d currentPose = drive.getPose();
            Pose2d resetPose = new Pose2d(
                    new Translation2d(Inches.of(651.22).in(Meters), Inches.of(317.69).in(Meters)),//new Translation2d(currentPose.getX(), currentPose.getY()),
                    (DriverStation.getAlliance().get() == Alliance.Red) ? Rotation2d.k180deg : Rotation2d.kZero);
            drive.resetGyro(resetPose);
        }));
        controller.povUp().onTrue(new InstantCommand(() -> {
            Pose2d resetPose = new Pose2d(
                    new Translation2d(
                        Units.inchesToMeters(29 / 2) + Units.inchesToMeters(13 / 4),
                            Units.inchesToMeters(29/2)),//158.32
                    (DriverStation.getAlliance().get() == Alliance.Red) ? Rotation2d.k180deg : Rotation2d.kZero);
            drive.resetGyro(resetPose);
        }));
        controller.povDown().whileTrue(drive.driveToPose(new Pose2d(14.6,4.75,Rotation2d.k180deg)).andThen(new DriveX(drive,-0.1).until(()->drive.getDetected())));
        // controller.leftBumper().onTrue(new InstantCommand(() -> {shooter.setRPS(40);}));
        // controller.rightBumper().onTrue(new InstantCommand(() -> {shooter.setRPS(0);}));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autos.getCommand();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
