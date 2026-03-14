package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MirrorUtil;

public class AutonConstants {
    public final static Pose2d startingMiddlePose = new Pose2d(3.466,3.976,Rotation2d.kZero);
    public final static Pose2d startingLeftPose = new Pose2d(3.450,2.388,Rotation2d.kCW_90deg.div(2));
    public static final Pose2d startingRightPose = MirrorUtil.mirrorPose(startingLeftPose);
}
