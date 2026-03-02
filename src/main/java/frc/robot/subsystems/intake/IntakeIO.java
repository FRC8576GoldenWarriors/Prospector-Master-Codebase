package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {}

  @AutoLog
 public class IntakeIOInputs {
    public Voltage pivotVoltage = Volts.of(0.0);
    public Current pivotCurrent = Amps.of(0);
     public Current pivotSupplyCurrent = Amps.of(0);
    public Current rollerSupplyCurrent = Amps.of(0);
    public AngularVelocity pivotRPS = RotationsPerSecond.of(0);
   // public Temperature pivotMotorTemperature = Celsius.of(0);
    public boolean pivotConnected = false;
    public Voltage rollerVoltage = Volts.of(0.0);
    public Current rollerCurrent = Amps.of(0);
    public AngularVelocity rollerRPS = RotationsPerSecond.of(0);
    //public Temperature rollerMotorTemperature = Celsius.of(0);
    public boolean rollerConnected = false;

    public boolean leftEncoderConnected = false;
    public boolean rightEncoderConnected = false;

    public double leftEncoderRotations = 0.0;

    public double rightEncoderRotations = 0.0;


}

    default void setPivotVoltage(double voltage) {}

    default void setPivotSpeed(double speed) {}

    default void setRollerVoltage(double voltage) {}

    default void setRollerSpeed(double speed) {}

    default boolean leftEncoderConnected() {
    return false;
     }

     default boolean rightEncoderConnected() {
    return false;
     }

}
