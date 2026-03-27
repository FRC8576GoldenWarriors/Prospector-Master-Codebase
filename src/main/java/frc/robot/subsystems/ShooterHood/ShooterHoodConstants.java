package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterHoodConstants {
    public static final int shooterHoodMotorID = 40;
    public static final int shooterHoodEncoderDIO = 3;
    public static double expectedZero = +0.92+0.06-0.03+0.65;//0.97+0.98+0.08-0.06-0.04+0.66+0.44-0.902;
    public static final boolean encoderInverted = false;

    public static final double kp = 4.5;//9.5;
    public static final double ki = 0.0;
    public static final double kd = 0.04;//0.04
    public static Constraints profile = new Constraints(1.2, 1.5);

    public static final double ks = 0.03;//0.15
    public static final double kg = 0.235;//0.25
    public static final double kv = 0.0;



    public static final double ka = 0.0;




}
