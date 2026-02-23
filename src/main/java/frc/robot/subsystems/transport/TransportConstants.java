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

    // Untuned
    public static final double kP = 10;//0.11;
    public static final double kI = 0;//0.5;
    public static final double kD = 0;//.001;
    public static final double kV = 0.12;


    public static final InvertedValue transportMotorInversion = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue transportMotorNeutralMode = NeutralModeValue.Brake;

    public static final Current transportMotorCurrentLimit = Amps.of(40);
    public static final boolean enableTransportMotorCurrentLimit = true;

    public static final AngularVelocity transportInSpeed = RPM.of(2000);
    public static final AngularVelocity transportOutSpeed = RPM.of(-2000);
}
