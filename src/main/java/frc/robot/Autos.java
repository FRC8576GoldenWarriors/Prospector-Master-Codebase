package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        PathPlannerPath path = (DriverStation.getAlliance().get()==Alliance.Blue)?PathPlannerPath.fromPathFile("ForwardTest"):flipPathWithoutHeading(PathPlannerPath.fromPathFile("ForwardTest"));
        return Commands.sequence(
          Commands.runOnce(() -> drive.resetOdometry(path.getStartingHolonomicPose().get())),
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
}
