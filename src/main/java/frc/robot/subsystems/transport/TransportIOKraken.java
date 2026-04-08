package frc.robot.subsystems.transport;


import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class TransportIOKraken implements TransportIO {
    private final TalonFX transportMotor;
    private final TalonFX transportMotorBack;
    private final TalonFXConfiguration transportMotorConfiguration;

    private final DigitalInput leftTransportPhotoElectric;
    private final DigitalInput rightTransportPhotoElectric;

    private final Debouncer leftTransportPhotoElectricDebouncer;
    private final Debouncer rightTransportPhotoElectricDebouncer;

    private final StatusSignal<Current> statorCurrentStatusSignal;
    private final StatusSignal<Current> supplyCurrentStatusSignal;

    private final StatusSignal<AngularVelocity> transportAngularVelocity;
    private final StatusSignal<Voltage> transportMotorVoltage;

    private final StatusSignal<Current> statorCurrentStatusSignalBack;
    private final StatusSignal<Current> supplyCurrentStatusSignalBack;

    private final StatusSignal<AngularVelocity> transportAngularVelocityBack;
    private final StatusSignal<Voltage> transportMotorVoltageBack;


    private final VelocityVoltage velcoityRequest = new VelocityVoltage(0);

    public TransportIOKraken() {
        transportMotor = new TalonFX(TransportConstants.transportMotorID);
        transportMotorBack = new TalonFX(TransportConstants.transportMotorID2);
        transportMotorConfiguration = new TalonFXConfiguration();
        transportMotorConfiguration.withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(TransportConstants.transportMotorNeutralMode)
            .withInverted(TransportConstants.transportMotorInversion))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(TransportConstants.transportMotorCurrentLimit)
            .withSupplyCurrentLimitEnable(TransportConstants.enableTransportMotorCurrentLimit)
        );
        leftTransportPhotoElectricDebouncer = new Debouncer(TransportConstants.debounceTime.in(Seconds), DebounceType.kFalling);
        rightTransportPhotoElectricDebouncer = new Debouncer(TransportConstants.debounceTime.in(Seconds), DebounceType.kFalling);



        Slot0Configs slot0Configs = transportMotorConfiguration.Slot0;
        slot0Configs.kV = TransportConstants.kV;
        slot0Configs.kP = TransportConstants.kP;
        slot0Configs.kI = TransportConstants.kI;
        slot0Configs.kD = TransportConstants.kD;


        statorCurrentStatusSignal = transportMotor.getStatorCurrent();
        supplyCurrentStatusSignal = transportMotor.getSupplyCurrent();

        transportAngularVelocity = transportMotor.getVelocity();
        transportMotorVoltage = transportMotor.getMotorVoltage();

        statorCurrentStatusSignalBack = transportMotorBack.getStatorCurrent();
        supplyCurrentStatusSignalBack = transportMotorBack.getSupplyCurrent();

        transportAngularVelocityBack = transportMotorBack.getVelocity();
        transportMotorVoltageBack = transportMotorBack.getMotorVoltage();

        transportMotor.optimizeBusUtilization(Hertz.of(0));

        BaseStatusSignal.setUpdateFrequencyForAll(
            TransportConstants.updateFrequency,
            statorCurrentStatusSignal,
            supplyCurrentStatusSignal,
            transportAngularVelocity,
            transportMotorVoltage
        );

        leftTransportPhotoElectric = new DigitalInput(TransportConstants.leftTransportPhotoelectricID);
        rightTransportPhotoElectric = new DigitalInput(TransportConstants.rightTransportPhotoelectricID);

        transportMotor.getConfigurator().apply(transportMotorConfiguration);
        transportMotorBack.getConfigurator().apply(transportMotorConfiguration);
        transportMotorBack.setControl(new Follower(TransportConstants.transportMotorID,MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(TransportIOInputs inputs) {
        StatusSignal.refreshAll(
            statorCurrentStatusSignal,
            supplyCurrentStatusSignal,
            transportAngularVelocity,
            transportMotorVoltage,
            statorCurrentStatusSignalBack,
            supplyCurrentStatusSignalBack,
            transportAngularVelocityBack,
            transportMotorVoltageBack);

        inputs.transportMotorIsConnected = transportMotor.isConnected();

        inputs.leftFuelDetected = leftTransportPhotoElectricDebouncer.calculate(!leftTransportPhotoElectric.get());
        inputs.rightFuelDetected = rightTransportPhotoElectricDebouncer.calculate(!rightTransportPhotoElectric.get());

        inputs.transportMotorStatorCurrent = statorCurrentStatusSignal.getValue();
        inputs.transportMotorSupplyCurrent = supplyCurrentStatusSignal.getValue();

        inputs.transportAngularVelocity = transportAngularVelocity.getValue();
        inputs.transportMotorVoltage = transportMotorVoltage.getValue();

        inputs.transportMotorStatorCurrentBack = statorCurrentStatusSignalBack.getValue();
        inputs.transportMotorSupplyCurrentBack = supplyCurrentStatusSignalBack.getValue();

        inputs.transportAngularVelocityBack = transportAngularVelocityBack.getValue();
        inputs.transportMotorVoltageBack = transportMotorVoltageBack.getValue();

    }

    @Override
    public void setTransportSpeed(AngularVelocity rpsVelocity) {
        transportMotor.setControl(velcoityRequest.withVelocity(rpsVelocity));
    }


    @Override
    @AutoLogOutput(key = "Transport/Voltage")
    public void setTransportVoltage(double volts) {
        transportMotor.setVoltage(volts);
    }

    @Override
    public void setkP(double kP){
        Slot0Configs PIDF = transportMotorConfiguration.Slot0;
        PIDF.kP = kP;
        transportMotor.getConfigurator().apply(transportMotorConfiguration);
    }


    @Override
    public void setkV(double kV){
        Slot0Configs PIDF = transportMotorConfiguration.Slot0;
        PIDF.kV = kV;
        transportMotor.getConfigurator().apply(transportMotorConfiguration);

    }

}
