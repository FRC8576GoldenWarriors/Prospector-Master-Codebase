package frc.robot.util;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class PoseUtil {

    /**
     *
     * @param x X translation as taken from onshape's coordinate system (+X rightward of the robot)
     * @param y Y translation as taken from onshape's coordinate system (+Y forward vector of the robot)
     * @param z Z translation as taken from onshape's coordinate system (+Z downward vector of the robot)
     * @param roll Roll as taken from onshape's coordinate system (Rotation about the X axis)
     * @param pitch Pitch as taken from onshape's coordinate system (Rotation about the Y axis)
     * @param yaw Yaw as taken from onshape's coordinate system (Rotation about the Z axis)
     * @return A Pose3d object with the given translation and rotation, converted from onshape's coordinate system to Limelight's coordinate system
     */

    public static Pose3d toLimelightPose(Distance x, Distance y, Distance z, Angle roll, Angle pitch, Angle yaw) {
        return new Pose3d(new Translation3d(y, x, z.baseUnit().zero().minus(z)), new Rotation3d(roll, pitch, yaw));
    }

}
