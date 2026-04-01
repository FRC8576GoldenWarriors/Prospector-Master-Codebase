package frc.robot;


import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Macros.RobotStates;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class Autos {
    private Drive drive;
    private Macros macros;
    private boolean flipped;
    private final LoggedDashboardChooser<Command> autoChooser;
    public Autos(Drive drive, Macros macros, boolean flipped){
        this.drive = drive;
        this.macros = macros;
        // ! Try running Right Side Steal Shoot unflipped to see if autobuilder will automatically flip it
        this.flipped = flipped;
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Left Depot Auton", leftAuton());
        // TODO: Clean up old autos
        // autoChooser.addOption("Right Leave Shoot", rightLeave());
        // autoChooser.addOption("Middle Leave Shoot", testAutonThingy());
        // autoChooser.addOption("Left Steal", leftSteal());

        //TODO: Test new autobuilder autos
        autoChooser.addOption("Right Side Steal Shoot", SideSwipeSteal(false));
        autoChooser.addOption("Middle Leave Shoot", MiddleLeaveShoot(flipped));
        autoChooser.addOption("Left Side Steal Shoot", SideSwipeSteal(true));
        SmartDashboard.putData("Auto Chooser",autoChooser.getSendableChooser());
    }

    public Command SideSwipeSteal(boolean isMirrored) {
        try {
            return
                getAutoBuilderPathPlannerCommand("OverBump", flipped, isMirrored)
            .alongWith(macros.setWantedState(RobotStates.IntakeOn))
            .andThen(getAutoBuilderPathPlannerCommand("FRIntakeBalls", flipped, isMirrored))
            .andThen(getAutoBuilderPathPlannerCommand("BackToBump", flipped, isMirrored, new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
            .andThen(getAutoBuilderPathPlannerCommand("BackOverBump", flipped, isMirrored)
            .alongWith(macros.setWantedState(RobotStates.IntakeOff)))
            .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())
            .until(()->DriveCommands.angleController.atGoal()))
            .andThen(macros.setWantedState(RobotStates.RunContinous));
        } catch (Exception exception) {
            return Commands.none();
        }
    }

    public Command MiddleLeaveShoot(boolean isMirrored) {
        try {
            return
                getAutoBuilderPathPlannerCommand("Rand", flipped, isMirrored)
            .withDeadline(new WaitCommand(3))
            .andThen(new InstantCommand(()->macros.setWantedStatePrivate(RobotStates.AutonShoot),macros));
        } catch (Exception exception) {
            return Commands.none();
        }
    }

    public Command rightLeave(){
        return runPath("OverBump", flipped,AutonConstants.startingRightPose)
        .alongWith(macros.setWantedState(RobotStates.IntakeOn))
        .andThen(runPath("FRIntakeBalls", flipped,new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
        .andThen(runPath("BackToBump", flipped,new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI)))
        .andThen(runPath("BackOverBump", flipped).alongWith(macros.setWantedState(RobotStates.IntakeOff)))
        .andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()))
        .andThen(macros.setWantedState(RobotStates.RunContinous));//andThen(runPath("IntakeBalls", flipped,new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).//.raceWith(new WaitCommand(8)).andThen(runPath("OverBump", flipped));//.andThen(runPath("OverBump", flipped));//runPath("RightLeave", flipped,AutonConstants.startingRightPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("OverBump",false,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn));//.andThen(runPath("IntakeBalls",false)).andThen(runPath("BackToBump",false)).andThen(runPath("BackOverBump",false)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }

    public Command leftAuton(){
        return runPath("LeftLeave", flipped,AutonConstants.startingLeftPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose())).until(()->DriveCommands.angleAligned()).andThen(macros.setWantedState(RobotStates.AutonShoot).withDeadline(new WaitCommand(6))).andThen(runPath("LeftToDepot", flipped).alongWith(macros.setWantedState(RobotStates.IntakeOn))).andThen(runPath("LeftOutOfDepot", flipped)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleAligned())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("LeftToDepot", flipped,AutonConstants.startingLeftPose).alongWith(macros.setWantedState(RobotStates.IntakeOn)).andThen(runPath("OutFromDepot", flipped)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.Shoot));//runPath("OverBump",true,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn)).andThen(runPath("IntakeBalls",true)).andThen(runPath("BackToBump",true)).andThen(runPath("BackOverBump",true)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }
    public Command leftSteal(){
        return runPath("OverBump", !flipped).alongWith(macros.setWantedState(RobotStates.IntakeOn)).andThen(runPath("IntakeBalls", !flipped,new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).andThen(runPath("FRIntakeBalls", !flipped,new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).andThen(runPath("BackToBump", !flipped,new PathConstraints(4.0, 5.2, 3*Math.PI, 4*Math.PI))).andThen(runPath("BackOverBump", !flipped).alongWith(macros.setWantedState(RobotStates.IntakeOff))).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.RunContinous));//.raceWith(new WaitCommand(8)).andThen(runPath("OverBump", flipped));//.andThen(runPath("OverBump", flipped));//runPath("RightLeave", flipped,AutonConstants.startingRightPose).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal())).andThen(macros.setWantedState(RobotStates.AutonShoot));//runPath("OverBump",false,AutonConstants.startingRightPose).alongWith(macros.setWantedState(RobotStates.IntakeOn));//.andThen(runPath("IntakeBalls",false)).andThen(runPath("BackToBump",false)).andThen(runPath("BackOverBump",false)).andThen(DriveCommands.joystickDriveTagCentric(drive,()->0,()->0,()->drive.getPose()).until(()->DriveCommands.angleController.atGoal()).andThen(macros.setWantedState(RobotStates.AutonShoot)));
    }
    public Command testAutonThingy(){
        return runPath("Rand",false,AutonConstants.startingMiddlePose).withDeadline(new WaitCommand(3)).andThen(new InstantCommand(()->macros.setWantedStatePrivate(RobotStates.AutonShoot),macros));//.andThen(macros.setWantedState(RobotStates.Idle));
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

    public Command getAutoBuilderPathPlannerCommand(String name, boolean flipped, boolean mirror) throws Exception {
        return AutoBuilder.followPath(this.getAutoBuilderPathPlannerPath(name, flipped, mirror));
    }

    public Command getAutoBuilderPathPlannerCommand(String name, boolean flipped, boolean mirror, PathConstraints constraints) throws Exception {
        return AutoBuilder.followPath(this.getAutoBuilderPathPlannerPath(name, flipped, mirror, constraints));
    }

    public PathPlannerPath getAutoBuilderPathPlannerPath(String pathName, boolean flipped, boolean mirror) throws Exception {
        return this.getAutoBuilderPathPlannerPath(pathName, flipped, mirror, null, null);
    }

    public PathPlannerPath getAutoBuilderPathPlannerPath(String pathName, boolean flipped, boolean mirror, PathConstraints constraints) throws Exception {
        return this.getAutoBuilderPathPlannerPath(pathName, flipped, mirror, constraints, null);
    }

    public PathPlannerPath getAutoBuilderPathPlannerPath(String pathName, boolean flipped, boolean mirror, PathConstraints constraints, Pose2d startingPose) throws Exception {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if(startingPose != null) {
            List<Waypoint> points = path.getWaypoints();

            points.set(
                0,
                new Waypoint(
                    null,
                    startingPose.getTranslation(),
                    points.get(1).anchor()));

            path = new PathPlannerPath(
                points,
                constraints,
                new IdealStartingState(
                    path.getIdealStartingState().velocityMPS(),
                    startingPose.getRotation()),
                    path.getGoalEndState());
        }

        if(flipped) {
            path = path.flipPath();
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
        if(flipped){
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
        else if(flipped&&mirrored){
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
        else if(!flipped&&mirrored){
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
        if(flipped){
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
        else if(flipped&&mirrored){
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
        else if(!flipped&&mirrored){
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
        if(flipped){
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
        else if(flipped&&mirrored){
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
        else if(!flipped&&mirrored){
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
}
