package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
       AngularVelocity transportAngularVelocity = RotationsPerSecond.of(0.0);


        boolean transportMotorIsConnected = false;

        boolean leftFuelDetected = false;
        boolean rightFuelDetected = false;
    }

    default void updateInputs(TransportIOInputs inputs) {}

    default void setTransportVoltage(double volts) {}
    default void setkP(double kP){}
    default void setkV(double kV){}

    default void setTransportSpeed(AngularVelocity rpsVelocity) {}
}
