package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    // Motor IDs (Subset 30-40)
    public static final int LEFT_SHOOTER_ID = 31;
    public static final int RIGHT_SHOOTER_ID = 32;

    // Current Limits
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

    // PIDFF Constants (untuned)
    public static final double kP = 0.11;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.12;
    public static final double kS = 0.24;

    // Manual Testing Constants
    public static final double MANUAL_STEP_RPM = 10.0;

    // Hardware Constants
    public static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(27.25); // When the shooter pivot is in the top position

    // Measurment Constants
    public static final double RADIUS_OF_HUB_METERS = Units.inchesToMeters(41.73);
    public static final double netHeight = Units.inchesToMeters(72) - SHOOTER_HEIGHT_METERS;

}
