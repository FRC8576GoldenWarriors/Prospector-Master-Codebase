package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeKrakenIO implements IntakeIO{

    public TalonFX pivotMotor;
    public TalonFX rollerMotor;
    private DutyCycleEncoder leftEncoder;
    private DutyCycleEncoder rightEncoder;
    TalonFXConfiguration leadConfig;
    TalonFXConfiguration pivotConfig;
    TalonFXConfiguration rollerConfig;

    private final StatusSignal<Voltage> pivotMotorVoltageSignal;
    private final StatusSignal<Current> pivotMotorStatorCurrentSignal;
    private final StatusSignal<Current> pivotMotorSupplyCurrentSignal;
    private final StatusSignal<AngularVelocity> pivotMotorAngularVelocitySignal;
    private final StatusSignal<Angle> pivotMotorAngleSignal;

    private final StatusSignal<Voltage> rollerMotorVoltageSignal;
    private final StatusSignal<Current> rollerMotorStatorCurrentSignal;
    private final StatusSignal<Current> rollerMotorSupplyCurrentSignal;
    private final StatusSignal<AngularVelocity> rollerMotorAngularVelocitySignal;
    private final StatusSignal<Angle> rollerMotorAngleSignal;


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
            .withSupplyCurrentLimit(Amps.of(60))
            // .withStatorCurrentLimit(Amps.of(40))
            // .withStatorCurrentLimitEnable(true)
            );


        rollerConfig = leadConfig.clone().withMotorOutput(
            leadConfig.MotorOutput.clone()
            .withInverted(IntakeConstants.Hardware.rollerInvertedValue)
            ).withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true).withStatorCurrentLimitEnable(false));//.withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true));

        pivotConfig = leadConfig.clone().withMotorOutput(
            leadConfig.MotorOutput.clone()
            .withInverted(IntakeConstants.Hardware.pivotInvert)
            );
        pivotConfig = pivotConfig.withCurrentLimits(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(60))
            .withSupplyCurrentLimitEnable(true));
            //withStatorCurrentLimitEnable(false));

        pivotMotorVoltageSignal = pivotMotor.getMotorVoltage();
        pivotMotorStatorCurrentSignal = pivotMotor.getStatorCurrent();
        pivotMotorSupplyCurrentSignal = pivotMotor.getSupplyCurrent();
        pivotMotorAngularVelocitySignal = pivotMotor.getVelocity();
        pivotMotorAngleSignal = pivotMotor.getPosition();

        rollerMotorVoltageSignal = rollerMotor.getMotorVoltage();
        rollerMotorStatorCurrentSignal = rollerMotor.getStatorCurrent();
        rollerMotorSupplyCurrentSignal = rollerMotor.getSupplyCurrent();
        rollerMotorAngularVelocitySignal = rollerMotor.getVelocity();
        rollerMotorAngleSignal = rollerMotor.getPosition();

        pivotMotor.optimizeBusUtilization(Hertz.of(0));
        rollerMotor.optimizeBusUtilization(Hertz.of(0));

        BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.Software.updateFrequency,
         pivotMotorVoltageSignal,
         pivotMotorStatorCurrentSignal,
         pivotMotorSupplyCurrentSignal,
         pivotMotorAngularVelocitySignal,
         pivotMotorAngleSignal,
         rollerMotorVoltageSignal,
         rollerMotorStatorCurrentSignal,
         rollerMotorSupplyCurrentSignal,
         rollerMotorAngularVelocitySignal,
         rollerMotorAngleSignal);



        pivotMotor.getConfigurator().apply(pivotConfig);
        rollerMotor.getConfigurator().apply(rollerConfig);
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
     BaseStatusSignal.refreshAll(
         pivotMotorVoltageSignal,
         pivotMotorStatorCurrentSignal,
         pivotMotorSupplyCurrentSignal,
         pivotMotorAngularVelocitySignal,
         pivotMotorAngleSignal,
         rollerMotorVoltageSignal,
         rollerMotorStatorCurrentSignal,
         rollerMotorSupplyCurrentSignal,
         rollerMotorAngularVelocitySignal,
         rollerMotorAngleSignal
     );

     inputs.pivotConnected = pivotMotor.isConnected();
     inputs.pivotVoltage = pivotMotorVoltageSignal.getValue();
     inputs.pivotCurrent = pivotMotorStatorCurrentSignal.getValue();
     inputs.pivotSupplyCurrent = pivotMotorSupplyCurrentSignal.getValue();
     inputs.pivotRPS = pivotMotorAngularVelocitySignal.getValue();

     inputs.rollerConnected = rollerMotor.isConnected();
     inputs.rollerVoltage = rollerMotorVoltageSignal.getValue();
     inputs.rollerCurrent = rollerMotorStatorCurrentSignal.getValue();
     inputs.rollerSupplyCurrent = rollerMotorSupplyCurrentSignal.getValue();
     inputs.rollerRPS = rollerMotorAngularVelocitySignal.getValue();

     inputs.leftEncoderConnected = leftEncoder.isConnected();
     inputs.leftEncoderRotations = Rotations.of(leftEncoder.get());

     inputs.rightEncoderConnected = rightEncoder.isConnected();
     inputs.rightEncoderRotations = Rotations.of(rightEncoder.get());
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
