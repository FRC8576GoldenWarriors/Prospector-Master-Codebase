package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public Voltage leftMotorVoltage = Volts.of(0.0);
    public Current leftMotorStatorCurrent = Amps.of(0.0);
    public Current leftMotorSupplyCurrent = Amps.of(0.0);
    public AngularVelocity leftMotorSpeed = RotationsPerSecond.of(0.0);
    public boolean leftMotorConnected = false;

    public Voltage rightMotorVoltage = Volts.of(0.0);
    public Current rightMotorStatorCurrent = Amps.of(0.0);
    public Current rightMotorSupplyCurrent = Amps.of(0.0);
    public AngularVelocity rightMotorSpeed = RotationsPerSecond.of(0.0);
    public boolean rightMotorConnected = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
  public default void setShooterVelocity(AngularVelocity leftVel, AngularVelocity rightVel) {}
  public default void setShooterVoltage(Voltage leftVolts, Voltage rightVolts) {}
  public default void setkP(double kP){}
  public default void stop() {}
}
