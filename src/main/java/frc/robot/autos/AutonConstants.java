package frc.robot.autos;


import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;

public class AutonConstants {
    public final static Pose2d startingMiddlePose = new Pose2d(3.466,3.976,Rotation2d.kZero);
    public static final Pose2d startingLeftPose = new Pose2d(3.633,5.504,Rotation2d.kZero);//new Pose2d(3.450,2.388,Rotation2d.kZero);/
    //public final static Pose2d startingLeftPose = new Pose2d(3.644,5.912,Rotation2d.k180deg);//new Pose2d(3.450,2.388,Rotation2d.kCCW_90deg.div(2));
    public static final Pose2d startingRightPose = new Pose2d(3.450,2.388,Rotation2d.fromDegrees(45));//MirrorUtil.mirrorPose(startingLeftPose);

    public static final boolean useFlipping = false;

    public static class PathPlannerConstants {

        public static final PIDConstants pathplannerTranslationPID = new PIDConstants(7.5, 0.0, 0.0);
        public static final PIDConstants pathplannerRotationPID = new PIDConstants(5.0, 0.0, 0.0);

    }

    public static class ChoreoConstants {

        public static final PIDController xTranslationController
            = new PIDController(
                PathPlannerConstants.pathplannerTranslationPID.kP,
                PathPlannerConstants.pathplannerTranslationPID.kI,
                PathPlannerConstants.pathplannerTranslationPID.kD);

        public static final PIDController yTranslationController
            = new PIDController(
                PathPlannerConstants.pathplannerTranslationPID.kP,
                PathPlannerConstants.pathplannerTranslationPID.kI,
                PathPlannerConstants.pathplannerTranslationPID.kD);

        public static final ProfiledPIDController rotationController
            = new ProfiledPIDController(
                PathPlannerConstants.pathplannerRotationPID.kP,
                PathPlannerConstants.pathplannerRotationPID.kI,
                PathPlannerConstants.pathplannerRotationPID.kD,
                DriveCommands.ANGLE_CONSTRAINTS);


        static {
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
        }
    }
}
