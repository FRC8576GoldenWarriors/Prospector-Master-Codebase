package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class TransportIOKraken implements TransportIO {

    private final TalonFX transportMotor;
    private final TalonFXConfiguration transportMotorConfiguration;
    private final DigitalInput leftTransportPhotoElectric;
    private final DigitalInput rightTransportPhotoElectric;

    private final StatusSignal<Current> statorCurrentStatusSignal;
    private final StatusSignal<Current> supplyCurrentStatusSignal;

    private final VelocityVoltage velcoityRequest = new VelocityVoltage(0);

    public TransportIOKraken() {
        transportMotor = new TalonFX(TransportConstants.transportMotorID);
        transportMotorConfiguration = new TalonFXConfiguration();
        transportMotorConfiguration.withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(TransportConstants.transportMotorNeutralMode)
            .withInverted(TransportConstants.transportMotorInversion))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(TransportConstants.transportMotorCurrentLimit)
            .withStatorCurrentLimitEnable(TransportConstants.enableTransportMotorCurrentLimit)
        );

        Slot0Configs slot0Configs = transportMotorConfiguration.Slot0;
        slot0Configs.kV = TransportConstants.kV;
        slot0Configs.kP = TransportConstants.kP;
        slot0Configs.kI = TransportConstants.kI;
        slot0Configs.kD = TransportConstants.kD;


        statorCurrentStatusSignal = transportMotor.getStatorCurrent();
        supplyCurrentStatusSignal = transportMotor.getSupplyCurrent();

        leftTransportPhotoElectric = new DigitalInput(TransportConstants.leftTransportPhotoelectricID);
        rightTransportPhotoElectric = new DigitalInput(TransportConstants.rightTransportPhotoelectricID);

        transportMotor.getConfigurator().apply(transportMotorConfiguration);
    }

    @Override
    public void updateInputs(TransportIOInputs inputs) {
        StatusSignal.refreshAll(statorCurrentStatusSignal, supplyCurrentStatusSignal);
        inputs.transportMotorIsConnected = transportMotor.isConnected();

        inputs.leftFuelDetected = leftTransportPhotoElectric.get();
        inputs.rightFuelDetected = rightTransportPhotoElectric.get();

        inputs.transportMotorStatorCurrent = statorCurrentStatusSignal.getValue();
        inputs.transportMotorSupplyCurrent = supplyCurrentStatusSignal.getValue();

        inputs.transportAngularVelocity = transportMotor.getVelocity().getValue();


    }

    @Override
    public void setTransportSpeed(AngularVelocity rpsVelocity) {
        transportMotor.setControl(velcoityRequest.withVelocity(rpsVelocity));
    }

    @Override
    public void setTransportVoltage(Voltage volts) {
        transportMotor.setVoltage(volts.in(Volts));
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
