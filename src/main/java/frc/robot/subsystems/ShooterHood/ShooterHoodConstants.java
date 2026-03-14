package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterHoodConstants {
    public static final int shooterHoodMotorID = 40;
    public static final int shooterHoodEncoderDIO = 3;
    public static double expectedZero = 0.9024+.17+.97+.27-0.01+0.248+0.59;
    public static final boolean encoderInverted = false;

    public static final double kp = 9.5;//9.5;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static Constraints profile = new Constraints(1.2, 1.5);

    public static final double ks = 0.1;
    public static final double kg = 0.25;
    public static final double kv = 0.0;
    public static final double ka = 0.0;




}
