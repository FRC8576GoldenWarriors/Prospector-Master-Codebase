package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TransportConstants {

    public static final int transportMotorID = 20;
    public static final int leftTransportPhotoelectricID = 2;
    public static final int rightTransportPhotoelectricID = 6;

    public static final InvertedValue transportMotorInversion = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue transportMotorNeutralMode = NeutralModeValue.Brake;

    public static final Current transportMotorCurrentLimit = Amps.of(40);
    public static final boolean enableTransportMotorCurrentLimit = true;

    public static final Voltage transportInVoltage = Volts.of(10.0);
    public static final Voltage transportOutVoltage = Volts.of(-10.0);
}
