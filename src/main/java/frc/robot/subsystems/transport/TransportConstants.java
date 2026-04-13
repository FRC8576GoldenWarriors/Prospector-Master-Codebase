package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class TransportConstants {

    public static final Frequency updateFrequency = Hertz.of(50);
    public static final Time debounceTime = Seconds.of(0.2);

    public static final int transportMotorID = 20;
    public static final int transportMotorID2 = 21;
    public static final int leftTransportPhotoelectricID = 5;
    public static final int rightTransportPhotoelectricID = 6;

    // Untuned
    public static final double kP = 10;//0.11;
    public static final double kI = 0;//0.5;
    public static final double kD = 0;//.001;
    public static final double kV = 0.16;

    public static final double velocityTarget =65;//110;//60;//110;//110;//100;//110;//100;//120;//100;//70;//60;//50//25;//40;
    public static final double velocityTargetSlower = 25;


    public static final InvertedValue transportMotorBackInversion = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue transportMotorInversion = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue transportMotorNeutralMode = NeutralModeValue.Brake;

    public static final Current transportMotorCurrentLimit = Amps.of(60);
    public static final boolean enableTransportMotorCurrentLimit = true;

    public static final AngularVelocity transportInSpeed = RPM.of(2000);
    public static final AngularVelocity transportOutSpeed = RPM.of(-2000);
}
