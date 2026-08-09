package frc.robot.autos;


import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Macros;
import frc.robot.Macros.RobotStates;
import frc.robot.commands.DriveCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ChoreoUtil;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.fieldPosition;
import frc.robot.util.LocalADStarAK;

public class Autos {
    private final Drive drive;
    private final Macros macros;

    private AutoFactory autoFactory;
    private final LoggedDashboardChooser<Command> autoChooser;

    private final BooleanSupplier shouldFlip;

    public static RobotConfig pathplannerRobotConfig;

    @AutoLogOutput
    public static Pose2d startingPose = new Pose2d();

    @AutoLogOutput
    public static boolean inNeutralZone = false;


    private static final Alert pathplannerRobotConfigAlert = new Alert("Unable to load PathPlanner RobotConfig from GUI.", AlertType.kWarning);

    // All Choreo Trajectories

    static {
        try {
            pathplannerRobotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            pathplannerRobotConfigAlert.set(true);
            DriverStation.reportWarning("Unable to load PathPlanner RobotConfig from GUI.", e.getStackTrace());
        }
    }


    public Autos(Drive drive, Macros macros){
        this.drive = drive;
        this.macros = macros;

        boolean alliance = DriverStation.getAlliance().get() == Alliance.Red;
        Logger.recordOutput("AutoAllianceFlipper", alliance);
        this.shouldFlip = () -> alliance;

        configurePathPlanner(drive);
        configureChoreo(drive);

        this.autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand(), autoFactory.warmupCmd());

        addSysIDRoutines();
        addPathPlannerAutonRoutines();
        //addChoreoAutonRoutines();


        //SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
    }

    private Command getAlignCommand() {
        return Commands.run(() -> DriveCommands.joystickDriveTagCentric(drive, () -> 0, () -> 0, drive::getPose), drive).until(DriveCommands::angleAligned);
    }

    private Command getShooterCommand() {
        return Commands.runEnd(() -> macros.setWantedState(RobotStates.RunContinous),() -> macros.setWantedState(RobotStates.Idle), macros).withTimeout(Seconds.of(6));
    }

    private Command getIntakeCommand() {
        return Commands.run(() -> macros.setWantedState(RobotStates.IntakeOn), macros);
    }

    private final void configurePathPlanner(Drive drivetrain) {
        AutoBuilder.configure(
                drivetrain::getPose,
                drivetrain::resetOdometry,
                drivetrain::getChassisSpeeds,
                drivetrain::runPathPlannerVelocity,
                new PPHolonomicDriveController(
                    AutonConstants.PathPlannerConstants.pathplannerTranslationPID,
                    AutonConstants.PathPlannerConstants.pathplannerRotationPID),
                pathplannerRobotConfig,
                () -> AutonConstants.useFlipping, // Prevents PathPlanner from handling flipping
                drivetrain);

        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    }

    private final void configureChoreo(Drive drivetrain) {
        autoFactory =
            new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetOdometry,
                drivetrain::runChoreoVelocity,
                AutonConstants.useFlipping, // Prevents Choreo from handling flipping
                drivetrain,
                (traj, edge) -> {
                    Logger.recordOutput("Choreo/Active Trajectory", traj.getPoses());
                });
    }

    private final void addSysIDRoutines() {
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private final void addPathPlannerAutonRoutines() {
        //autoChooser.addOption("Left Depot Auton", leftAuton());
        autoChooser.addDefaultOption("Left Depot Auton", autoLeftDepot()); //
        autoChooser.addOption("Right Steal", autoRightLeave()); //
        autoChooser.addOption("Middle Leave Shoot", testAutonThingy()); // Good
        autoChooser.addOption("Left Steal", autoLeftLeave()); //
        autoChooser.addOption("Left Steal Long", autoLeftStealLong()); //
        autoChooser.addOption("Right Steal Long", autoRightStealLong()); //
        autoChooser.addOption("Right Double Dip", rightDoubleSteal()); // Good
        autoChooser.addOption("Right Full Side Auton", autoRightFull()); // Good
        autoChooser.addOption("Left Full Side Auton", autoLeftFull()); // Good
        autoChooser.addOption("Middle Depot", autoMiddleDepot()); //
        autoChooser.addOption("Disruption", disruption());
        autoChooser.addOption("Right Tear Drop", rightTearDrop());
        autoChooser.addOption("Left Tear Drop", leftTearDrop());
    }

    private final void addChoreoAutonRoutines() {
        String identifier = "[Choreo] ";
        autoChooser.addOption(identifier + "Right Single Steal", rightLeave(autoFactory).cmd());
        autoChooser.addOption(identifier + "Left Single Steal", leftLeave(autoFactory).cmd());

        autoChooser.addOption(identifier + "Right Double Steal", rightDoubleSteal(autoFactory).cmd());
        autoChooser.addOption(identifier + "Left Double Steal", leftDoubleSteal(autoFactory).cmd());

        autoChooser.addOption(identifier + "Middle Leave Shoot", middleLeaveShoot(autoFactory).cmd());

        autoChooser.addOption(identifier + "Right Full Pass", rightFullPass(autoFactory).cmd());
        autoChooser.addOption(identifier + "Left Full Pass", leftFullPass(autoFactory).cmd());

        autoChooser.addOption(identifier + "Middle Depot", middleDepot(autoFactory).cmd());
    }

    // public Command rightLeave(){
    //     return runPath("OverBump", shouldFlip.getAsBoolean(),AutonConstants.startingRightPose)
    //     .alongWith(macros.setWantedState(RobotStates.IntakeOn))
    //     .andThen(runPath("FRIntakeBalls", shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
    //     .andThen(runPath("BackToBump", shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
    //     .andThen(runPath("BackOverBump", shouldFlip.getAsBoolean()).alongWith(macros.setWantedState(RobotStates.IntakeOff)))
    //     .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()))
    //     .andThen(macros.setWantedState(RobotStates.RunContinous).withDeadline(new WaitCommand(6)));
    //     //.andThen();//andThen(runPath("IntakeBalls", shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).//.raceWith(new WaitCommand(8)).andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//.andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//runPath("RightLeave", shouldFlip.getAsBoolean(),AutonConstants.startingRightPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("OverBump",false,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn));//.andThen(runPath("IntakeBalls",false)).andThen(runPath("BackToBump",false)).andThen(runPath("BackOverBump",false)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    // }

    public AutoRoutine rightLeave(AutoFactory autoFactory) {
        String routineName = "Right Leave";

        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory overBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory leave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "Leave");
        AutoTrajectory backOverBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump");

        routine.active().onTrue(
          Commands.sequence(
            overBump.resetOdometry(),
            overBump.cmd()
          )
        );

        overBump.active().onTrue(getIntakeCommand());

        overBump.chain(leave);
        leave.chain(backOverBump);

        backOverBump.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;
    }

    public AutoRoutine leftLeave(AutoFactory autoFactory) {
        String routineName = "Left Leave";

        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory overBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory leave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "Leave");
        AutoTrajectory backOverBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump");

        routine.active().onTrue(
          Commands.sequence(
            overBump.resetOdometry(),
            overBump.cmd()
          )
        );

        overBump.active().onTrue(getIntakeCommand());

        overBump.chain(leave);
        leave.chain(backOverBump);

        backOverBump.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;
    }

    public AutoRoutine rightDoubleSteal(AutoFactory autoFactory) {
        String routineName = "Right Double Steal";

        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory overBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory leave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "Leave");
        AutoTrajectory backOverBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump");
        AutoTrajectory hubLeave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "HubLeave");
        AutoTrajectory overBumpAgain = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory backOverBumpAgain = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump");

        routine.active().onTrue(
            Commands.sequence(
                overBump.resetOdometry(),
                overBump.cmd()
            )
        );

        overBump.active().onTrue(getIntakeCommand());

        overBump.chain(leave);
        leave.chain(backOverBump);

        backOverBump.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand(),
                overBumpAgain.cmd()
            )
        );

        overBumpAgain.chain(hubLeave);
        hubLeave.chain(backOverBumpAgain);

        backOverBumpAgain.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;

    }

    public AutoRoutine leftDoubleSteal(AutoFactory autoFactory) {
        String routineName = "Left Double Steal";

        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory overBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory leave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "Leave");
        AutoTrajectory backOverBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump");
        AutoTrajectory hubLeave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "HubLeave");
        AutoTrajectory overBumpAgain = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory backOverBumpAgain = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump");

        routine.active().onTrue(
            Commands.sequence(
                overBump.resetOdometry(),
                overBump.cmd()
            )
        );

        overBump.active().onTrue(getIntakeCommand());

        overBump.chain(leave);
        leave.chain(backOverBump);

        backOverBump.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand(),
                overBumpAgain.cmd()
            )
        );

        overBumpAgain.chain(hubLeave);
        hubLeave.chain(backOverBumpAgain);

        backOverBumpAgain.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;

    }

    public AutoRoutine middleLeaveShoot(AutoFactory autoFactory) {
        String routineName = "Middle Leave Shoot";
        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory middleLeave = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "MiddleLeave");

        routine.active().onTrue(
            Commands.sequence(
                middleLeave.resetOdometry(),
                middleLeave.cmd()
            )
        );

        middleLeave.done().onTrue(getShooterCommand());

        return routine;

    }

    public AutoRoutine rightFullPass(AutoFactory autoFactory) {
        String routineName = "Right Full Pass";
        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory overBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory fullPass = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "FullPass");
        AutoTrajectory backOverBump = ChoreoUtil.flipAcrossMidline(ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump"), true);

        routine.active().onTrue(
            Commands.sequence(
                overBump.resetOdometry(),
                overBump.cmd()
            )
        );

        overBump.active().onTrue(
            getIntakeCommand()
        );

        overBump.chain(fullPass);
        fullPass.chain(backOverBump);

        backOverBump.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;

    }

    public AutoRoutine leftFullPass(AutoFactory autoFactory) {
        String routineName = "Left Full Pass";
        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory overBump = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "OverBump");
        AutoTrajectory fullPass = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "FullPass");
        AutoTrajectory backOverBump = ChoreoUtil.flipAcrossMidline(ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "BackOverBump"), true);

        routine.active().onTrue(
            Commands.sequence(
                overBump.resetOdometry(),
                overBump.cmd()
            )
        );

        overBump.active().onTrue(
            getIntakeCommand()
        );

        overBump.chain(fullPass);
        fullPass.chain(backOverBump);

        backOverBump.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;

    }

    public AutoRoutine leftRightTest(AutoFactory autofactory) {
        String routineName = "LRTest";
        AutoRoutine routine = autofactory.newRoutine(routineName);

        AutoTrajectory leftRightTest = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "LeftRightTest");
        routine.active().onTrue(
            Commands.sequence(
                leftRightTest.resetOdometry(),
                leftRightTest.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine frontBackTurnTest(AutoFactory autofactory) {
        String routineName = "FBTTest";
        AutoRoutine routine = autofactory.newRoutine(routineName);

        AutoTrajectory frontBackTurnTest = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "FrontBackTurnTest");
        routine.active().onTrue(
            Commands.sequence(
                frontBackTurnTest.resetOdometry(),
                frontBackTurnTest.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine arcFollowTest(AutoFactory autofactory) {
        String routineName = "AFTest";
        AutoRoutine routine = autofactory.newRoutine(routineName);

        AutoTrajectory arcFollowTest = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "ArcFollowTest");
        routine.active().onTrue(
            Commands.sequence(
                arcFollowTest.resetOdometry(),
                arcFollowTest.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine middleDepot(AutoFactory autoFactory) {
        String routineName = "Middle Depot";
        AutoRoutine routine = autoFactory.newRoutine(routineName);

        AutoTrajectory middleDepot = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "MiddleDepot");
        AutoTrajectory DepotScore = ChoreoUtil.loadAndFlipMidlineAndDiagonal(routine, routineName, "DepotScore");

        routine.active().onTrue(
            Commands.sequence(
                middleDepot.resetOdometry(),
                middleDepot.cmd()
            )
        );

        middleDepot.active().onTrue(
            getIntakeCommand()
        );

        middleDepot.chain(DepotScore);

        DepotScore.done().onTrue(
            Commands.sequence(
                getAlignCommand(),
                getShooterCommand()
            )
        );

        return routine;
    }

    public Command disruption() {
        try {
            return getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, null)
            .andThen(getAutoBuilderPathPlannerCommand("Disrupt", shouldFlip.getAsBoolean(), true, null))
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), true, null))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            .until(()->DriveCommands.angleController.atGoal()))
            .andThen(macros.setWantedState(RobotStates.RunContinous))
            .alongWith(new WaitCommand(6));
        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }

    public Command rightTearDrop() {
        try {
            return getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderChoreoCommand("Leave", shouldFlip.getAsBoolean(), false, null))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, null))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            .until(()->DriveCommands.angleController.atGoal()))
            .andThen(macros.setWantedState(RobotStates.RunContinous))
            .alongWith(new WaitCommand(6));
        } catch (Exception e) {
             DriverStation.reportError(e.getLocalizedMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command leftTearDrop() {
        try {
            return getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, null)
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderChoreoCommand("Leave", shouldFlip.getAsBoolean(), true, null))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), true, null))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            .until(()->DriveCommands.angleController.atGoal()))
            .andThen(macros.setWantedState(RobotStates.RunContinous))
            .alongWith(new WaitCommand(6));
        } catch (Exception e) {
             DriverStation.reportError(e.getLocalizedMessage(), e.getStackTrace());
            return Commands.none();
        }
    }



    public Command rightDoubleSteal(){
        try {
        return getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
                .alongWith(macros.setWantedState(RobotStates.IntakeOn))
                .andThen(getAutoBuilderPathPlannerCommand("IntakeBalls", shouldFlip.getAsBoolean(), false, null))
                .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), false, null))
                .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, null))
                .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
                .until(()->DriveCommands.angleController.atGoal()))
                .andThen(macros.setWantedState(RobotStates.RunContinous)
                .alongWith(new WaitCommand(6)))
                .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
                .alongWith(macros.setWantedState(RobotStates.IntakeOn)))
                .andThen(getAutoBuilderPathPlannerCommand("IntakeHub", shouldFlip.getAsBoolean(), false, null))
                .andThen(getAutoBuilderPathPlannerCommand("HubOverBump", shouldFlip.getAsBoolean(), false, null))
                .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
                .until(()->DriveCommands.angleController.atGoal()))
                .andThen(macros.setWantedState(RobotStates.RunContinous)
                .alongWith(new WaitCommand(6)));
        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
     }
    public Command autoMiddleDepot(){
        try {
            return
            getAutoBuilderPathPlannerCommand("MiddleDepot", shouldFlip.getAsBoolean(), false, new PathConstraints(2, 3.4, 3*Math.PI, 4*Math.PI))
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("OutFromDepot", shouldFlip.getAsBoolean(), false, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            //.alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous)
             .alongWith(DriveCommands.driveX(drive, () -> new ChassisSpeeds())));
            //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
            //  .andThen(macros.setWantedState(RobotStates.Rest))//.alongWith(new WaitCommand(1)))
            //  .andThen(getAutoBuilderPathPlannerCommand("LeftToDepot", shouldFlip.getAsBoolean(), false, new PathConstraints(2, 5.2, 3*Math.PI, 4*Math.PI))
            //  .alongWith(macros.setWantedState(RobotStates.IntakeOn)))
            //  .andThen(getAutoBuilderPathPlannerCommand("LeftOutOfDepot", shouldFlip.getAsBoolean(), false, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            //  .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            //  .until(()->DriveCommands.angleAligned()))
            //  .andThen(macros.setWantedState(RobotStates.RunContinous));//.alongWith(new WaitCommand(6)));

        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }
    public Command autoRightFull(){
        try{
        return getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
                .alongWith(macros.setWantedState(RobotStates.IntakeOn))
                .andThen(getAutoBuilderPathPlannerCommand("IntakeBallsFull", shouldFlip.getAsBoolean(), false, null))
                .andThen(getAutoBuilderPathPlannerCommand("BackToBumpFull", shouldFlip.getAsBoolean(), false, null))
                .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), true, null))
                .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
                .until(()->DriveCommands.angleController.atGoal()))
                .andThen(macros.setWantedState(RobotStates.RunContinous));
        }catch(Exception exception){
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }
    public Command autoLeftFull(){
        try{
        return getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, null)
                .alongWith(macros.setWantedState(RobotStates.IntakeOn))
                .andThen(getAutoBuilderPathPlannerCommand("IntakeBallsFull", shouldFlip.getAsBoolean(), true, null))
                .andThen(getAutoBuilderPathPlannerCommand("BackToBumpFull", shouldFlip.getAsBoolean(), true, null))
                .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, null))
                .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
                .until(()->DriveCommands.angleController.atGoal()))
                .andThen(macros.setWantedState(RobotStates.RunContinous));
        }catch(Exception exception){
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }
    public Command autoLeftDepot(){
        try {
            return
            getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, new PathConstraints(4.2, 5.2, 3*Math.PI, 4*Math.PI))
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("IntakeBalls", shouldFlip.getAsBoolean(), true, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), true, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), true, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), true, new PathConstraints(4.2, 5.2, 3*Math.PI, 4*Math.PI)))
            //.alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(4))
             .alongWith(DriveCommands.driveX(drive, () -> new ChassisSpeeds(), Seconds.of(4))))
            //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
             .andThen(macros.setWantedState(RobotStates.Rest))//.alongWith(new WaitCommand(1)))
             .andThen(getAutoBuilderPathPlannerCommand("LeftToDepot", shouldFlip.getAsBoolean(), false, new PathConstraints(2, 5.2, 3*Math.PI, 4*Math.PI))
             .alongWith(macros.setWantedState(RobotStates.IntakeOn)))
             .andThen(getAutoBuilderPathPlannerCommand("LeftOutOfDepot", shouldFlip.getAsBoolean(), false, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
             .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous));//.alongWith(new WaitCommand(6)));

        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }
    public Command autoLeftLeave(){
        try {
            return
                getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, new PathConstraints(4.2, 5.2, 3*Math.PI, 4*Math.PI))
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("IntakeBalls", shouldFlip.getAsBoolean(), true, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), true, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), true, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), true, new PathConstraints(4.2, 5.2, 3*Math.PI, 4*Math.PI)))
            //.alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(6))
             .alongWith(DriveCommands.driveX(drive, () -> new ChassisSpeeds(), Seconds.of(6))))
            //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
             .andThen(macros.setWantedState(RobotStates.Rest).alongWith(new WaitCommand(1)))
             .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, null).alongWith(macros.setWantedState(RobotStates.IntakeOn)))
             .andThen(getAutoBuilderPathPlannerCommand("IntakeHub", shouldFlip.getAsBoolean(), true, null));

        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }
    public Command autoLeftStealLong(){
        try {
            return
                getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, null)
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), true, null))
            .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), true, null))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), true, new PathConstraints(3, 4.5, 3*Math.PI, 4*Math.PI))
            .alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(6))
             .alongWith(DriveCommands.driveX(drive, () -> new ChassisSpeeds(), Seconds.of(6))))
            //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
             .andThen(macros.setWantedState(RobotStates.Rest).alongWith(new WaitCommand(1)))
             .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), true, null))
             .andThen(macros.setWantedState(RobotStates.IntakeOn))
             .andThen(getAutoBuilderPathPlannerCommand("Intake2", shouldFlip.getAsBoolean(), true, null));

        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }
    }

    public Command autoRightLeave() {
        try {
            // return
            //     getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
            // .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            // .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), false, null))
            // .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), false, null))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            // .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, new PathConstraints(3, 4.5, 3*Math.PI, 4*Math.PI))
            // .alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            // .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            //  .until(()->DriveCommands.angleAligned()))
            //  .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(6)))
            // //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
            //  .andThen(macros.setWantedState(RobotStates.Rest).alongWith(new WaitCommand(1)))
            //  .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
            //  .andThen(getAutoBuilderPathPlannerCommand("IntakeHub", shouldFlip.getAsBoolean(), false, null));
            return
                getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, new PathConstraints(4.2, 5.2, 3*Math.PI, 4*Math.PI))
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("IntakeBalls", shouldFlip.getAsBoolean(), false, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), false, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), false, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, new PathConstraints(4.2, 5.2, 3*Math.PI, 4*Math.PI)))
            //.alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(6))
             .alongWith(DriveCommands.driveX(drive, () -> new ChassisSpeeds(), Seconds.of(6))))
            //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
             .andThen(macros.setWantedState(RobotStates.Rest).alongWith(new WaitCommand(1)))
             .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null).alongWith(macros.setWantedState(RobotStates.IntakeOn)))
             .andThen(getAutoBuilderPathPlannerCommand("IntakeHub", shouldFlip.getAsBoolean(), false, null));

             // .alongWith(macros.setWantedState(RobotStates.IntakeOn));
        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }//andThen(runPath("IntakeBalls", shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).//.raceWith(new WaitCommand(8)).andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//.andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//runPath("RightLeave", shouldFlip.getAsBoolean(),AutonConstants.startingRightPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("OverBump",false,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn));//.andThen(runPath("IntakeBalls",false)).andThen(runPath("BackToBump",false)).andThen(runPath("BackOverBump",false)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }
    public Command autoRightStealLong() {
        try {
            // return
            //     getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
            // .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            // .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), false, null))
            // .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), false, null))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            // .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, new PathConstraints(3, 4.5, 3*Math.PI, 4*Math.PI))
            // .alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            // .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            //  .until(()->DriveCommands.angleAligned()))
            //  .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(6)))
            // //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
            //  .andThen(macros.setWantedState(RobotStates.Rest).alongWith(new WaitCommand(1)))
            //  .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
            //  .andThen(getAutoBuilderPathPlannerCommand("IntakeHub", shouldFlip.getAsBoolean(), false, null));
            return
                getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null)
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("IntakeBalls", shouldFlip.getAsBoolean(), false, null))
            .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", shouldFlip.getAsBoolean(), false, null))
            .andThen(getAutoBuilderPathPlannerCommand("BackToBump", shouldFlip.getAsBoolean(), false, null))//new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", shouldFlip.getAsBoolean(), false, new PathConstraints(3, 4.5, 3*Math.PI, 4*Math.PI)))
            //.alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
             .until(()->DriveCommands.angleAligned()))
             .andThen(macros.setWantedState(RobotStates.RunContinous).alongWith(new WaitCommand(6))
             .alongWith(DriveCommands.driveX(drive, () -> new ChassisSpeeds(), Seconds.of(6))))
            //.andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
             .andThen(macros.setWantedState(RobotStates.Rest).alongWith(new WaitCommand(1)))
             .andThen(macros.setWantedState(RobotStates.IntakeOn))
             .andThen(getAutoBuilderPathPlannerCommand("OverBump", shouldFlip.getAsBoolean(), false, null))
             .andThen(getAutoBuilderPathPlannerCommand("Intake2", shouldFlip.getAsBoolean(), false, new PathConstraints(2, 3.5, 3*Math.PI, 4*Math.PI)));
             // .alongWith(macros.setWantedState(RobotStates.IntakeOn));
        } catch (Exception exception) {
            DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
            return Commands.none();
        }//andThen(runPath("IntakeBalls", shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).//.raceWith(new WaitCommand(8)).andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//.andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//runPath("RightLeave", shouldFlip.getAsBoolean(),AutonConstants.startingRightPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("OverBump",false,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn));//.andThen(runPath("IntakeBalls",false)).andThen(runPath("BackToBump",false)).andThen(runPath("BackOverBump",false)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }


    public Command leftAuton(){
        return runPath("LeftLeave", shouldFlip.getAsBoolean(),AutonConstants.startingLeftPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())).until(()->DriveCommands.angleAligned()).andThen(macros.setWantedState(RobotStates.AutonShoot).withDeadline(new WaitCommand(6))).andThen(runPath("LeftToDepot", shouldFlip.getAsBoolean()).alongWith(macros.setWantedState(RobotStates.IntakeOn))).andThen(runPath("LeftOutOfDepot", shouldFlip.getAsBoolean())).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleAligned())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("LeftToDepot", shouldFlip.getAsBoolean(),AutonConstants.startingLeftPose).alongWith(macros.setWantedState(RobotStates.IntakeOn)).andThen(runPath("OutFromDepot", shouldFlip.getAsBoolean())).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.Shoot));//runPath("OverBump",true,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn)).andThen(runPath("IntakeBalls",true)).andThen(runPath("BackToBump",true)).andThen(runPath("BackOverBump",true)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }
    public Command leftSteal(){
        return runPath("OverBump", !shouldFlip.getAsBoolean()).alongWith(macros.setWantedState(RobotStates.IntakeOn)).andThen(runPath("IntakeBalls", !shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).andThen(runPath("FRIntakeBalls", !shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).andThen(runPath("BackToBump", !shouldFlip.getAsBoolean(),new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).andThen(runPath("BackOverBump", !shouldFlip.getAsBoolean()).alongWith(macros.setWantedState(RobotStates.IntakeOff))).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.RunContinous));//.raceWith(new WaitCommand(8)).andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//.andThen(runPath("OverBump", shouldFlip.getAsBoolean()));//runPath("RightLeave", shouldFlip.getAsBoolean(),AutonConstants.startingRightPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("OverBump",false,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn));//.andThen(runPath("IntakeBalls",false)).andThen(runPath("BackToBump",false)).andThen(runPath("BackOverBump",false)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }
    public Command testAutonThingy(){
        return runPath("Rand",false,AutonConstants.startingMiddlePose).withDeadline(new WaitCommand(3)).andThen(new InstantCommand(()->macros.setWantedStatePrivate(RobotStates.RunContinous),macros));//.andThen(macros.setWantedState(RobotStates.Idle));
        // PathPlannerPath path =(PathPlannerPath.fromPathFile("Rand")).flipPath();//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        // //path.preventFlipping = true;
        // // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        // //     path = path.flipPath();
        // // }
        // Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
        // return new SequentialCommandGroup(
        //    new InstantCommand(()->drive.resetOdometry(new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()))),
        //   drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation().plus(Rotation2d.k180deg)))),//AutoBuilder.followPath(path));
    }
    public Command teleDrive(){
        return drive.driveToPose((shouldFlip.getAsBoolean())?FlippingUtil.flipFieldPose(new Pose2d(0.405,3.750,Rotation2d.kCW_90deg)):new Pose2d(0.405,3.750,Rotation2d.kCW_90deg)).alongWith(macros.setWantedState(RobotStates.IntakeOn))
        .andThen(drive.driveToPose((shouldFlip.getAsBoolean())?FlippingUtil.flipFieldPose(new Pose2d(0.427,2.028,Rotation2d.kCW_90deg)):new Pose2d(0.427,2.028,Rotation2d.kCW_90deg)))
        .andThen(drive.driveToPose((shouldFlip.getAsBoolean())?FlippingUtil.flipFieldPose(new Pose2d(2.266,2.147,Rotation2d.kZero)):new Pose2d(2.266,2.147,Rotation2d.kZero)))
        .andThen(DriveCommands.joystickDriveTagCentric(drive, ()->0, ()->0, drive::getPose))
        .until(()->DriveCommands.angleAlignedLEDs());
    }

    public Command driveOverBump(boolean onRightSide, boolean onAllianceSide){
        try{
            return getAutoBuilderPathPlannerCommand("OverBump", onAllianceSide, !onRightSide, new PathConstraints(4.2, 5.0, 3*Math.PI, 4*Math.PI));

    }catch(Exception exception){
        DriverStation.reportError(exception.getLocalizedMessage(), exception.getStackTrace());
        return Commands.none();
    }
    }

    public Command driveBackOverBump(boolean onRightSide, boolean onAllianceSide){
       // inNeutralZone = FieldUtil.getFieldPosition(startingPose)==fieldPosition.NeutralZone;

        try{

           return getAutoBuilderPathPlannerCommand("BackOverBump", onAllianceSide, !onRightSide, new PathConstraints(4.2, 5.0, 3*Math.PI, 4*Math.PI));

        }
        catch (Exception e) {
            DriverStation.reportError(e.getLocalizedMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command getAutoBuilderPathPlannerCommand(String name, boolean flipped, boolean mirror, PathConstraints constraints) throws Exception {
        return AutoBuilder.followPath(this.getAutoBuilderPathPlannerPath(name, flipped, mirror, constraints));
    }

    public PathPlannerPath getAutoBuilderPathPlannerPath(String pathName, boolean flipped, boolean mirror, PathConstraints constraints) throws Exception {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if(flipped) {
            path = path.flipPath();
           // path = path.mirrorPath();
        }

        if(mirror) {
            path = path.mirrorPath();
        }

        if(constraints != null)
            return new PathPlannerPath(path.getWaypoints(), constraints, path.getIdealStartingState(), path.getGoalEndState());
        else
            return path;
    }

    public Command getAutoBuilderChoreoCommand(String pathName, boolean flipped, boolean mirror, PathConstraints constraints) throws Exception {
        return AutoBuilder.followPath(this.getAutoBuilderChoreoPath(pathName, flipped, mirror, constraints));
    }

    public PathPlannerPath getAutoBuilderChoreoPath(String pathName, boolean flipped, boolean mirror, PathConstraints constraints) throws Exception {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
        if(flipped) {
            path = path.flipPath();
           // path = path.mirrorPath();
        }

        if(mirror) {
            path = path.mirrorPath();
        }

        if(constraints != null)
            return new PathPlannerPath(path.getWaypoints(), constraints, path.getIdealStartingState(), path.getGoalEndState());
        else
            return path;
    }

    public PathPlannerPath flipPathWithoutHeading(PathPlannerPath originalPath){
        if(originalPath!=null){
        List<PathPoint> pathpoints = originalPath.getAllPathPoints();
        List<PathPoint> endPoints = new ArrayList<PathPoint>();
        originalPath = originalPath.flipPath();
        List<PathPoint> flippedPath = originalPath.getAllPathPoints();
        for(int i =0;i<pathpoints.size();i++){
            endPoints.add(new PathPoint(
                flippedPath.get(i).position,
                pathpoints.get(i).rotationTarget,
                pathpoints.get(i).constraints
            ));
        }
        return PathPlannerPath.fromPathPoints(endPoints, originalPath.getGlobalConstraints(), originalPath.getGoalEndState());
    }
    return new PathPlannerPath(null, null, null, null);
    }

    public SequentialCommandGroup runPath(String pathName,boolean mirrored,Pose2d startingPose){
        if(shouldFlip.getAsBoolean()){
            try{
            PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).flipPath();
            Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
            return new SequentialCommandGroup(
                //new InstantCommand(()->drive.resetOdometry(FlippingUtil.flipFieldPose(startingPose))),
          drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation()))));
            }catch(Exception e){
                e.printStackTrace();
                return (SequentialCommandGroup) Commands.none();
            }
        }
        else if(shouldFlip.getAsBoolean()&&mirrored){
            try{
                //Will need to pass in the Starting pose of the mirrored side
            PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).flipPath().mirrorPath();
            Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
            return new SequentialCommandGroup(
                //new InstantCommand(()->drive.resetOdometry(MirrorUtil.mirrorPose(FlippingUtil.flipFieldPose(startingPose)))),
                drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation().plus(Rotation2d.k180deg)))));
            }catch(Exception e){
                e.printStackTrace();
                return (SequentialCommandGroup) Commands.none();
            }
        }
        else if(!shouldFlip.getAsBoolean()&&mirrored){
            try{
        PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).mirrorPath();//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        //path.preventFlipping = true;
        // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        //     path = path.flipPath();
        // }
        Logger.recordOutput("Odometry/Starting Pose", path.getStartingHolonomicPose().get());
        return new SequentialCommandGroup(
            //new InstantCommand(()->drive.resetOdometry(MirrorUtil.mirrorPose((startingPose)))),
          drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation())));
        }catch(Exception e){
            e.printStackTrace();
            return (SequentialCommandGroup) Commands.none();
        }
        }else{
            try{
        PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName));//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        //path.preventFlipping = true;
        // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        //     path = path.flipPath();
        // }
        Logger.recordOutput("Odometry/Starting Pose", path.getStartingHolonomicPose().get());
        return new SequentialCommandGroup(
           // new InstantCommand(()->drive.resetOdometry((startingPose))),
            drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation())));
        }catch(Exception e){
            e.printStackTrace();
            return (SequentialCommandGroup) Commands.none();
        }
        }
    }

    public SequentialCommandGroup runPath(String pathName, boolean mirrored){
        if(shouldFlip.getAsBoolean()){
            try{
            PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).flipPath();
            Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
            return new SequentialCommandGroup(
            //    new InstantCommand(()->drive.resetOdometry(FlippingUtil.flipFieldPose(startingPose))),
          drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation()))));
            }catch(Exception e){
                e.printStackTrace();
                return (SequentialCommandGroup) Commands.none();
            }
        }
        else if(shouldFlip.getAsBoolean()&&mirrored){
            try{
                //Will need to pass in the Starting pose of the mirrored side
            PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).flipPath().mirrorPath();
            Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
            return new SequentialCommandGroup(
                //new InstantCommand(()->drive.resetOdometry(MirrorUtil.mirrorPose(FlippingUtil.flipFieldPose(startingPose)))),
                drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation().plus(Rotation2d.k180deg)))));
            }catch(Exception e){
                e.printStackTrace();
                return (SequentialCommandGroup) Commands.none();
            }
        }
        else if(!shouldFlip.getAsBoolean()&&mirrored){
            try{
        PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).mirrorPath();//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        //path.preventFlipping = true;
        // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        //     path = path.flipPath();
        // }
        Logger.recordOutput("Odometry/Starting Pose", path.getStartingHolonomicPose().get());
        return new SequentialCommandGroup(
            //new InstantCommand(()->drive.resetOdometry(MirrorUtil.mirrorPose((startingPose)))),
          drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation())));
        }catch(Exception e){
            e.printStackTrace();
            return (SequentialCommandGroup) Commands.none();
        }
        }else{
            try{
        PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName));//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        //path.preventFlipping = true;
        // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        //     path = path.flipPath();
        // }
        Logger.recordOutput("Odometry/Starting Pose", path.getStartingHolonomicPose().get());
        return new SequentialCommandGroup(
           // new InstantCommand(()->drive.resetOdometry((startingPose))),
            drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation())));
        }catch(Exception e){
            e.printStackTrace();
            return (SequentialCommandGroup) Commands.none();
        }
        }
    }
    public SequentialCommandGroup runPath(String pathName, boolean mirrored,PathConstraints constraints){
        if(shouldFlip.getAsBoolean()){
            try{
            PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).flipPath();
            Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
            return new SequentialCommandGroup(
            //    new InstantCommand(()->drive.resetOdometry(FlippingUtil.flipFieldPose(startingPose))),
          drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation())),constraints));
            }catch(Exception e){
                e.printStackTrace();
                return (SequentialCommandGroup) Commands.none();
            }
        }
        else if(shouldFlip.getAsBoolean()&&mirrored){
            try{
                //Will need to pass in the Starting pose of the mirrored side
            PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).flipPath().mirrorPath();
            Logger.recordOutput("Odometry/Starting Pose", new Pose2d(path.getStartingHolonomicPose().get().getTranslation(),path.getStartingHolonomicPose().get().getRotation()));
            return new SequentialCommandGroup(
                //new InstantCommand(()->drive.resetOdometry(MirrorUtil.mirrorPose(FlippingUtil.flipFieldPose(startingPose)))),
                drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation().plus(Rotation2d.k180deg))),constraints));
            }catch(Exception e){
                e.printStackTrace();
                return (SequentialCommandGroup) Commands.none();
            }
        }
        else if(!shouldFlip.getAsBoolean()&&mirrored){
            try{
        PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName)).mirrorPath();//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        //path.preventFlipping = true;
        // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        //     path = path.flipPath();
        // }
        Logger.recordOutput("Odometry/Starting Pose", path.getStartingHolonomicPose().get());
        return new SequentialCommandGroup(
            //new InstantCommand(()->drive.resetOdometry(MirrorUtil.mirrorPose((startingPose)))),
          drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation()),constraints));
        }catch(Exception e){
            e.printStackTrace();
            return (SequentialCommandGroup) Commands.none();
        }
        }else{
            try{
        PathPlannerPath path =(PathPlannerPath.fromPathFile(pathName));//.flipPath();//(DriverStation.getAlliance().get().equals(Alliance.Red))? PathPlannerPath.fromPathFile("Rand").flipPath():PathPlannerPath.fromPathFile("Rand");//getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        //path.preventFlipping = true;
        // if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        //     path = path.flipPath();
        // }
        Logger.recordOutput("Odometry/Starting Pose", path.getStartingHolonomicPose().get());
        return new SequentialCommandGroup(
           // new InstantCommand(()->drive.resetOdometry((startingPose))),
            drive.driveToPose(new Pose2d(path.getAllPathPoints().get(path.getAllPathPoints().size()-1).position,path.getAllPathPoints().get(path.getAllPathPoints().size()-1).rotationTarget.rotation()),constraints));
        }catch(Exception e){
            e.printStackTrace();
            return (SequentialCommandGroup) Commands.none();
        }
        }
    }

    public Command getCommand(){
        return autoChooser.get();
    }
    public PathPlannerPath getFlippedPath(PathPlannerPath originalPath){
        List<PathPoint> newPathPoints = new ArrayList<PathPoint>();
        Rotation2d startingHeading = originalPath.getInitialHeading().plus(Rotation2d.k180deg);
        TrapezoidProfile rotProfile = new TrapezoidProfile(new Constraints(Units.degreesToRadians(540), Units.degreesToRadians(720)));
        TrapezoidProfile.State currentState = new TrapezoidProfile.State(MathUtil.angleModulus(startingHeading.getRadians()),0.0);

        for(PathPoint point: originalPath.getAllPathPoints()){
            //Rotation2d rotGoal = FlippingUtil.flipFieldRotation((point.rotationTarget.rotation()));
            Rotation2d rotGoal = FlippingUtil.flipFieldPosition((point.position)).getAngle();
            Rotation2d rotationSetpoint = new Rotation2d(rotProfile.calculate(point.distanceAlongPath, currentState, new TrapezoidProfile.State(MathUtil.angleModulus(rotGoal.getRadians() + Math.PI),0.0)).position);
            newPathPoints.add(new PathPoint(
                FlippingUtil.flipFieldPosition(point.position),
                new RotationTarget(point.waypointRelativePos, rotationSetpoint),
                point.constraints
            ));
        }

        return PathPlannerPath.fromPathPoints(newPathPoints,originalPath.getGlobalConstraints(), originalPath.getGoalEndState().flip());
    }
    public void setNeutralZone(){
        inNeutralZone = FieldUtil.getFieldPosition(startingPose)==fieldPosition.NeutralZone;
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
