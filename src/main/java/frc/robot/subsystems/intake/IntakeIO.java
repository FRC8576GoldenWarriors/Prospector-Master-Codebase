package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {}

  @AutoLog
 public class IntakeIOInputs {
    public double pivotVoltage = 0.0;
    public double pivotCurrent = 0.0;
        public double pivotSupplyCurrent = 0.0;
        public double rollerSupplyCurrent = 0.0;

    public double pivotRPS = 0.0;
    public boolean pivotConnected = false;
    public double rollerVoltage = 0.0;
    public double rollerCurrent = 0.0;
    public double rollerRPS = 0.0;
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
