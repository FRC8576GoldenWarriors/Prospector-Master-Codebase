package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;



public class ShooterUtil {
    InterpolatingTreeMap<Double, Double> speedMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    InterpolatingTreeMap<Double, Double> angleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    public ShooterUtil(){
        speedMap.put(2.7+0.55, 33.0);
        speedMap.put(2.5+0.55, 35.0);
        speedMap.put(1.02+.55, 28.0);
        speedMap.put(1.9+.55, 32.0);
        speedMap.put(4.0+.55, 40.0);
        speedMap.put(5.43+.55, 43.0);
        speedMap.put(3.8+.55, 36.0);
        speedMap.put(3.37+.55, 36.5);
        speedMap.put(1.7+.55, 32.0);
        speedMap.put(2.22+.55, 32.0);

        angleMap.put(2.7+.55, 0.1);
        angleMap.put(2.5+.55, 0.03);
        angleMap.put(1.02+.55, 0.0);
        angleMap.put(1.9+.55, 0.02);
        angleMap.put(4.0+.55, 0.1);
        angleMap.put(5.43+.55,0.15);
        angleMap.put(3.8+.55, 0.1);
        angleMap.put(3.37+.55, 0.07);
        angleMap.put(1.7+.55, 0.04);
        angleMap.put(2.22+.55, 0.07);
    }
    // public static AngularVelocity calculateShotVelocity(double limelightDistanceMeters, double hoodAngleDegrees) {
    //     //Variables
    //     double theta = Math.toRadians(90-hoodAngleDegrees);
    //     double rHub = ShooterConstants.RADIUS_OF_HUB_METERS;
    //     double Rmin = limelightDistanceMeters - rHub;
    //     double Rmax = limelightDistanceMeters + rHub;
    //     double avgRad = (1+2)/2;

    //     // Max and Min Velocity for shooting in hub
    //     double Vmin = Math.sqrt((9.81 * Math.pow(Rmin, 2)) / (2 * Math.pow(Math.cos(theta), 2) * (Rmin * Math.tan(theta) - netHeight)));
    //     double Vmax = Math.sqrt((9.81 * Math.pow(Rmax, 2)) / ( 2 *Math.pow(Math.cos(theta), 2) * (Rmax * Math.tan(theta) - netHeight)));

    //     Logger.recordOutput("VminRPS", Vmin/(2.0 * Math.PI * Units.inchesToMeters(avgRad)));
    //     Logger.recordOutput("VmaxRPS", Vmax/(2.0 * Math.PI * Units.inchesToMeters(avgRad)));

    //     // Average for best error accomodation
    //     double Vbest = (Vmin + Vmax) / 2.0;

    //     // Convert m/s to RPS
    //     double RPSBest = Vbest / (2.0 * Math.PI * Units.inchesToMeters(avgRad));
    //     Logger.recordOutput("VbestRPS", RPSBest);
    //     return RotationsPerSecond.of(RPSBest);
    // }
    public AngularVelocity getRPS(double limelightDistanceMeters){
        return RotationsPerSecond.of(speedMap.get(limelightDistanceMeters));
    }

    public double getAngle(double limelightDistanceMeters){
        return angleMap.get(limelightDistanceMeters);
    }
}
