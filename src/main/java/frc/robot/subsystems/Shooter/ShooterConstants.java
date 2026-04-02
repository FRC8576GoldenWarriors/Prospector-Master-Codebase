package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    // Motor IDs (Subset 30-40)
    public static final int LEFT_SHOOTER_ID = 31;
    public static final int RIGHT_SHOOTER_ID = 32;

    // Current Limits
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

    // PIDFF Constants (untuned)
    public static final double kP = 0;//4;//1;//0;//5;//10;//0.11;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kVLeft = 0.125175*1.142;//0.12;//0.12;
    public static final double kALeft = 0.20611;
    public static final double kS = 0;//0.15;//0.24;
    public static final double kV = 0.14;//0.1375;//0.1275;//0.130018;//0.12*1.5;
    public static final double kARight = 0.17284;
    public static final double kPLeft = 0;//3;//0.17284;//3;
    // Manual Testing Constants
    public static final double MANUAL_STEP_RPM = 50.0;

    // Hardware Constants
    public static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(27.25); // When the shooter pivot is in the top position

    // Measurment Constants
    public static final double RADIUS_OF_HUB_METERS = Units.inchesToMeters(41.73);
    public static final double netHeight = Units.inchesToMeters(72) - SHOOTER_HEIGHT_METERS;

    public static final double shooterRPSFudge = -6.5;
    public static final double angleFudge = 0.01;
}
