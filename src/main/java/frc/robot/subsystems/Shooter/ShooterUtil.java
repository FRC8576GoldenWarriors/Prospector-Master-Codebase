package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;



public class ShooterUtil {
    InterpolatingTreeMap<Double, Double> speedMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    InterpolatingTreeMap<Double, Double> angleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    List<Double> distanceList = new ArrayList<>();

    private double speedFudge = 0;//8.5;//2.5;
    private double angleFudge = 0;//0.06;
    InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    public ShooterUtil(){
        speedMap.put(1.16, 32.62);
        speedMap.put(1.75, 33.12);
        speedMap.put(2.05, 33.12);
        speedMap.put(2.21, 33.82);
        speedMap.put(2.61, 35.65);
        speedMap.put(3.29, 38.12);
        speedMap.put(3.24, 37.93);
        speedMap.put(2.037, 36.02);
        speedMap.put(1.45, 30.87);
        //Video Points Day 1
        speedMap.put(1.828,33.91);
        speedMap.put(1.866, 34.29);
        speedMap.put(2.244, 33.976);
        speedMap.put(2.33, 34.36);
        speedMap.put(2.66, 35.84);


        // speedMap.put(1.76, 31.12);
        // speedMap.put(2.25, 34.5);
        // //Iffy points
        // speedMap.put(4.28, 37.0);
        // speedMap.put(2.4, 34.5);
        // speedMap.put(2.7+0.55, 33.0);
        // speedMap.put(2.5+0.55, 35.0);
        // speedMap.put(1.02+.55, 28.0);
        // speedMap.put(1.9+.55, 32.0);
        // speedMap.put(4.0+.55, 40.0);
        // speedMap.put(5.43+.55, 43.0);
        // speedMap.put(3.8+.55, 36.0);
        // speedMap.put(3.37+.55, 36.5);
        // speedMap.put(1.7+.55, 32.0);
        // speedMap.put(2.22+.55, 32.0);

        angleMap.put(1.16, 0.011);
        angleMap.put(1.75, 0.047);
        angleMap.put(2.05, 0.047);
        angleMap.put(2.61, 0.097);
        angleMap.put(3.29, 0.117);
        angleMap.put(3.24, 0.135);
        angleMap.put(2.037, 0.067);
        angleMap.put(2.21, 0.0814);
        angleMap.put(1.45, 0.029);
        //Video points day 1
        angleMap.put(1.828,0.052);
        angleMap.put(1.866, 0.055);
        angleMap.put(2.244, 0.083);
        angleMap.put(2.33, 0.086);
        angleMap.put(2.66, 0.10018);


        // angleMap.put(1.76,0.0113);
        // angleMap.put(2.25, 0.07);
        // //Iffy points
        // angleMap.put(2.4, 0.07);
        // angleMap.put(2.7+.55, 0.1);
        // angleMap.put(2.5+.55, 0.03);
        // angleMap.put(1.02+.55, 0.0);
        // angleMap.put(1.9+.55, 0.02);
        // angleMap.put(4.0+.55, 0.1);
        // angleMap.put(5.43+.55,0.15);
        // angleMap.put(3.8+.55, 0.1);
        // angleMap.put(3.37+.55, 0.07);
        // angleMap.put(1.7+.55, 0.04);
        // angleMap.put(2.22+.55, 0.07);

        distanceList.add(2.7+.55);
        distanceList.add(2.5+.55);
        distanceList.add(1.02+.55);
        distanceList.add(1.9+.55);
        distanceList.add(4.0+.55);
        distanceList.add(5.43+.55);
        distanceList.add(3.8+.55);
        distanceList.add(3.37+.55);
        distanceList.add(1.7+.55);
        distanceList.add(2.22+.55);

        tofMap.put(3.11, 1.05);
        tofMap.put(1.9, null);
        tofMap.put(4.27, null);
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

    public void fudgeSpeed(double speedFactor){
        speedFudge += speedFactor;
    }
    public void angleFudge(double angleFactor){
        angleFudge +=angleFactor;
    }

    public void resetSpeedFudge(){
        speedFudge = 0;
    }
    public void resetAngleFudge(){
        angleFudge = 0;
    }
    @AutoLogOutput (key = "ShooterUtil/Calculated RPS")
    public AngularVelocity getRPS(double limelightDistanceMeters){
        //Logger.recordOutput("ShooterUtil/Calculated RPS", RotationsPerSecond.of(MathUtil.clamp(speedMap.get(limelightDistanceMeters)+speedFudge, 0, 87)));
        return RotationsPerSecond.of(MathUtil.clamp(speedMap.get(limelightDistanceMeters)+speedFudge, 0, 87));
    }
    @AutoLogOutput (key = "ShooterUtil/Calculated Angle")
    public double getAngle(double limelightDistanceMeters){
        //Logger.recordOutput("ShooterUtil/Calculated RPS", RotationsPerSecond.of(MathUtil.clamp(speedMap.get(limelightDistanceMeters)+speedFudge, 0, 87)));
        return MathUtil.clamp(angleMap.get(limelightDistanceMeters)+angleFudge, 0, 0.37);
    }

    public double getNearestDistance(double distance) {
        var nearest = distanceList.stream()
                .min((d1, d2) -> Double.compare(Math.abs(d1 - distance), Math.abs(d2 - distance)))
                .orElse(0.0);
        return nearest;
    }

    // public Pair<AngularVelocity, Double> getSOTFCalc(double limelightDistanceX, double limelightDistanceY,double chassisVelX, double chassisVelY){
    //     double tof = tofMap.get(Math.hypot(limelightDistanceX, limelightDistanceY));
    //     for(int i =0;i<10;i++){
    //         limelightDistanceX+=chassisVelX*tof;
    //         limelightDistanceY+=chassisVelY*tof;
    //         tof = tofMap.get(Math.hypot(limelightDistanceX, limelightDistanceY));
    //     }
    //     return Pair.of(getRPS(Math.hypot(limelightDistanceX, limelightDistanceY)), getAngle(Math.hypot(limelightDistanceX, limelightDistanceY)));

    // }
    // public statci void main - Lalitha
}
