package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class TransportConstants {

    public static final int transportMotorID = 20;
    public static final int leftTransportPhotoelectricID = 2;
    public static final int rightTransportPhotoelectricID = 6;

    public static final InvertedValue transportMotorInversion = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue transportMotorNeutralMode = NeutralModeValue.Brake;

    public static final Current transportMotorCurrentLimit = Amps.of(40);
    public static final boolean enableTransportMotorCurrentLimit = true;

    public static final AngularVelocity transportInSpeed = RPM.of(2000);
    public static final AngularVelocity transportOutSpeed = RPM.of(-2000);
}
