package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeKrakenIO implements IntakeIO{

    public TalonFX pivotMotor;
    public TalonFX rollerMotor;
    private DutyCycleEncoder leftEncoder;
    private DutyCycleEncoder rightEncoder;
    TalonFXConfiguration leadConfig;
    TalonFXConfiguration pivotConfig;
    TalonFXConfiguration rollerConfig;


    public IntakeKrakenIO(){
        pivotMotor = new TalonFX(IntakeConstants.Hardware.pivotID);
        rollerMotor = new TalonFX(IntakeConstants.Hardware.rollerID);

        leftEncoder = new DutyCycleEncoder(IntakeConstants.Hardware.leftEncoderID,1.0, IntakeConstants.Software.leftZero);
        leftEncoder.setInverted(IntakeConstants.Software.leftInverted);
        rightEncoder = new DutyCycleEncoder(IntakeConstants.Hardware.rightEncoderID,1.0, IntakeConstants.Software.rightZero);
        rightEncoder.setInverted(IntakeConstants.Software.rightInverted);

        leadConfig = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40))
            .withStatorCurrentLimitEnable(true)
            );


        rollerConfig = leadConfig.clone().withMotorOutput(
            leadConfig.MotorOutput.clone()
            .withInverted(IntakeConstants.Hardware.rollerInvertedValue)
            ).withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true));

        pivotConfig = leadConfig.clone().withMotorOutput(
            leadConfig.MotorOutput.clone()
            .withInverted(IntakeConstants.Hardware.pivotInvert)

            );


        pivotMotor.getConfigurator().apply(pivotConfig);
        rollerMotor.getConfigurator().apply(rollerConfig);
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
     inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValue();
     inputs.pivotCurrent = pivotMotor.getStatorCurrent().getValue();
     inputs.pivotRPS = pivotMotor.getVelocity().getValue();
     //inputs.pivotMotorTemperature = pivotMotor.getDeviceTemp().getValue();
     inputs.pivotConnected = pivotMotor.isConnected();
     inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValue();
     inputs.rollerCurrent = rollerMotor.getStatorCurrent().getValue();
     inputs.rollerRPS = rollerMotor.getVelocity().getValue();
     //inputs.rollerMotorTemperature = rollerMotor.getDeviceTemp().getValue();
     inputs.rollerConnected = rollerMotor.isConnected();
     inputs.leftEncoderConnected = leftEncoder.isConnected();
     inputs.rightEncoderConnected = rightEncoder.isConnected();
     inputs.leftEncoderRotations = leftEncoder.get();
     inputs.rightEncoderRotations = rightEncoder.get();
     inputs.pivotSupplyCurrent = pivotMotor.getSupplyCurrent().getValue();
     inputs.rollerSupplyCurrent = rollerMotor.getSupplyCurrent().getValue();
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);//voltage
    }


    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);//speed
    }



}
