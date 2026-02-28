package frc.robot.subsystems.ShooterHood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public interface ShooterHoodIO {
    default void updateInputs(ShooterHoodInputs inputs) {}

    @AutoLog
    class ShooterHoodInputs {
        public Voltage voltage = Volts.of(0.0);
        public Current statorCurrent = Amps.of(0.0);
        public Current supplyCurrent = Amps.of(0.0);
        public AngularVelocity speed = RotationsPerSecond.of(0);
        public boolean isMotorConnected = false;
        public Angle encoderValue_Rotations = Rotations.of(0);
        public boolean isEncoderConnected = false;
    }

    default void setVoltage(double voltage) {}

    default void setSpeed(double speed) {}
}
