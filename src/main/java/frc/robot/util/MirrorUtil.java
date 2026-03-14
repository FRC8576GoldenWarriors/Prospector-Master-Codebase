package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.vision.VisionConstants;

public class MirrorUtil {
    public static Pose2d mirrorPose(Pose2d initialPose2d){
        if(initialPose2d.getY()+VisionConstants.aprilTagLayout.getFieldWidth()/2>VisionConstants.aprilTagLayout.getFieldWidth()){
            return initialPose2d.plus(new Transform2d(Meters.of(0), Meters.of(-VisionConstants.aprilTagLayout.getFieldWidth()/2), Rotation2d.kZero));
        }
        return initialPose2d.plus(new Transform2d(Meters.of(0), Meters.of(VisionConstants.aprilTagLayout.getFieldWidth()/2), Rotation2d.kZero));
    }
}
