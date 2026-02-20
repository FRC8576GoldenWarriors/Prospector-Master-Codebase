package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimberKrakenIO implements ClimberIO{

    public TalonFX pivotMotor;
    public TalonFX rollerMotor;
    private DutyCycleEncoder encoder;

    TalonFXConfiguration leadConfig;

    private CANrange backRightCANRange;
    private CANrange backLeftCANRange;
    private CANrange frontRightANRange;
    private CANrange frontLeftCANRange;
    private CANrange rightFacingCANRange;

    DigitalInput photoelectric;


    public ClimberKrakenIO(){
        pivotMotor = new TalonFX(ClimberConstants.Hardware.pivotID);

        encoder = new DutyCycleEncoder(ClimberConstants.Hardware.encoderID, 1.0, ClimberConstants.Software.encoderZero);
        encoder.setInverted(ClimberConstants.Software.encoderInvert);


        leadConfig = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake).withInverted(ClimberConstants.Hardware.pivotInvert)
            )
            .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60))
            .withStatorCurrentLimitEnable(true)
            );
        photoelectric = new DigitalInput(ClimberConstants.Hardware.photoelectricID);
        backRightCANRange = new CANrange(ClimberConstants.Hardware.backRightCANRange) ;
        backLeftCANRange = new CANrange(ClimberConstants.Hardware.backLeftCANRange);
        frontRightANRange = new CANrange(ClimberConstants.Hardware.frontRightANRange);
        frontLeftCANRange = new CANrange(ClimberConstants.Hardware.frontLeftCANRange);
        rightFacingCANRange = new CANrange(ClimberConstants.Hardware.rightFacingCANRange);
        pivotMotor.getConfigurator().apply(leadConfig);
    }


    @Override
    public void updateInputs(ClimberIOInputs inputs) {
     inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValue();
     inputs.pivotCurrent = pivotMotor.getStatorCurrent().getValue();
     inputs.pivotRPS = pivotMotor.getVelocity().getValue();
     //inputs.pivotMotorTemperature = pivotMotor.getDeviceTemp().getValue();
     inputs.pivotConnected = pivotMotor.isConnected();

     inputs.encoderConnected = encoder.isConnected();
     inputs.encoderRotations = encoder.get();
     inputs.pivotSupplyCurrent = pivotMotor.getSupplyCurrent().getValue();


    inputs.backRightCANRangeConnected =      backRightCANRange.isConnected();
    inputs.backLeftCANRangeConnected =      backLeftCANRange.isConnected();
    inputs.frontRightANRangeConnected =      frontRightANRange.isConnected();
    inputs.frontLeftCANRangeConnected =      frontLeftCANRange.isConnected();
    inputs.rightFacingCANRangeConnected =    rightFacingCANRange.isConnected();


    inputs.backRightCANRangeDistance =   backRightCANRange.getDistance().getValueAsDouble();
    inputs.backLeftCANRangeDistance =    backLeftCANRange.getDistance().getValueAsDouble();
    inputs.frontRightANRangeDistance =   frontRightANRange.getDistance().getValueAsDouble();
    inputs.frontLeftCANRangeDistance =   frontLeftCANRange.getDistance().getValueAsDouble();
    inputs.rightFacingCANRangeDistance =   rightFacingCANRange.getDistance().getValueAsDouble();

    inputs.photoelectricDetected = photoelectric.get();
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }



}
