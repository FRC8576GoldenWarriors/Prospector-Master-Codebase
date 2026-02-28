package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterHoodConstants {
    public static final int shooterHoodMotorID = 40;
    public static final int shooterHoodEncoderDIO = 3;
    public static double expectedZero = -0.6+0.28;//0;
    public static final boolean encoderInverted = false;

    public static final double kp = 3;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static Constraints profile = new Constraints(0.6, 0.75);

    public static final double ks = 0.0;
    public static final double kg = 0.3;
    public static final double kv = 0.0;
    public static final double ka = 0.0;




}
