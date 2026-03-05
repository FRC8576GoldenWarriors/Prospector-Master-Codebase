package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import static frc.robot.subsystems.Shooter.ShooterConstants.netHeight;

import org.littletonrobotics.junction.Logger;

public class ShooterUtil {


    public static AngularVelocity calculateShotVelocity(double limelightDistanceMeters, double hoodAngleDegrees) {
        //Variables
        double theta = Math.toRadians(90 - hoodAngleDegrees);
        double rHub = ShooterConstants.RADIUS_OF_HUB_METERS;
        double Rmin = limelightDistanceMeters - rHub;
        double Rmax = limelightDistanceMeters + rHub;

        // Max and Min Velocity for shooting in hub
        double Vmin = Math.sqrt((9.81 * Math.pow(Rmin, 2)) / (2 * Math.pow(Math.cos(theta), 2) * (Rmin * Math.tan(theta) - netHeight)));
        double Vmax = Math.sqrt((9.81 * Math.pow(Rmax, 2)) / (2 * Math.pow(Math.cos(theta), 2) * (Rmax * Math.tan(theta) - netHeight)));

        Logger.recordOutput("VminRPS", Vmin/(2.0 * Math.PI * Units.inchesToMeters(3)));
        Logger.recordOutput("VmaxRPS", Vmax/(2.0 * Math.PI * Units.inchesToMeters(3)));

        // Average for best error accomodation
        double Vbest = (Vmin + Vmax) / 2.0;

        // Convert m/s to RPS
        double RPSBest = Vbest / (2.0 * Math.PI * Units.inchesToMeters(3));
        Logger.recordOutput("VbestRPS", RPSBest);
        return RotationsPerSecond.of(RPSBest);
    }
}
