package frc.robot.subsystems.ShooterHood;

import static frc.robot.subsystems.Shooter.ShooterConstants.netHeight;


public class ShooterHoodUtil {

    public static double calculateHoodAngleDegrees(double limelightDistanceMeters) {
        return 90-Math.toDegrees(Math.atan2((netHeight + Math.sqrt(Math.pow(netHeight, 2)+ Math.pow(limelightDistanceMeters, 2))),limelightDistanceMeters));
    }

}
