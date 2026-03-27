package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonConstants {
    public final static Pose2d startingMiddlePose = new Pose2d(3.466,3.976,Rotation2d.kZero);
    public static final Pose2d startingLeftPose = new Pose2d(3.633,5.504,Rotation2d.kZero);//new Pose2d(3.450,2.388,Rotation2d.kZero);/
    //public final static Pose2d startingLeftPose = new Pose2d(3.644,5.912,Rotation2d.k180deg);//new Pose2d(3.450,2.388,Rotation2d.kCCW_90deg.div(2));
    public static final Pose2d startingRightPose = new Pose2d(3.450,2.388,Rotation2d.fromDegrees(45));//MirrorUtil.mirrorPose(startingLeftPose);
}
