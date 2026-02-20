package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TransportIO {

    @AutoLog
    public class TransportIOInputs {
        Voltage transportMotorVoltage = Volts.of(0);
        Current transportMotorStatorCurrent = Amps.of(0);
        Current transportMotorSupplyCurrent = Amps.of(0);

        boolean transportMotorIsConnected = false;

        boolean leftFuelDetected = false;
        boolean rightFuelDetected = false;
    }

    default void updateInputs(TransportIOInputs inputs) {}

    default void setTransportVoltage(Voltage volts) {}

    default void setTransportSpeed(AngularVelocity rpsVelocity) {}
}
