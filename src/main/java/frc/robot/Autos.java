package frc.robot;


import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class Autos {
    private Drive drive;
    private final LoggedDashboardChooser<Command> autoChooser;
    public Autos(Drive drive){
        this.drive = drive;
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
        autoChooser.addOption("Auto Class Test", testAutonThingy());
        SmartDashboard.putData("Auto Chooser",autoChooser.getSendableChooser());
    }

    public Command testAutonThingy(){
        try{
        PathPlannerPath path = getFlippedPath(PathPlannerPath.fromPathFile("Random"));
        path.preventFlipping = true;
        return Commands.sequence(
           // Commands.run(()->drive.resetOdometry(path.getStartingHolonomicPose().get())),
          AutoBuilder.followPath(path));
        }catch(Exception e){
            e.printStackTrace();
            return Commands.none();

        }
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
