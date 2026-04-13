package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;



public class ShooterIOKraken implements ShooterIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX addedMotor;

  private TalonFXConfiguration config;
  private TalonFXConfiguration leftConfig;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private SysIdRoutine sysId;

  private final StatusSignal<Voltage> leftMotorVoltageSignal;
  private final StatusSignal<Current> leftMotorStatorCurrentSignal;
  private final StatusSignal<Current> leftMotorSupplyCurrentSignal;
  private final StatusSignal<AngularVelocity> leftMotorVelocitySignal;
  private final StatusSignal<Angle> leftEncoderPositionSignal;

  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> rightMotorStatorCurrentSignal;
  private final StatusSignal<Current> rightMotorSupplyCurrentSignal;
  private final StatusSignal<AngularVelocity> rightMotorVelocitySignal;
  private final StatusSignal<Angle> rightEncoderPositionSignal;


  private final StatusSignal<Voltage> addedMotorVoltageSignal;
  private final StatusSignal<Current> addedMotorStatorCurrentSignal;
  private final StatusSignal<Current> addedMotorSupplyCurrentSignal;
  private final StatusSignal<AngularVelocity> addedMotorVelocitySignal;
  private final StatusSignal<Angle> addedEncoderPositionSignal;

  public ShooterIOKraken() {
    leftMotor = new TalonFX(ShooterConstants.LEFT_SHOOTER_ID);
    rightMotor = new TalonFX(ShooterConstants.RIGHT_SHOOTER_ID);
    addedMotor = new TalonFX(ShooterConstants.ADDED_SHOOTER_ID);
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Current Limits & Brake Mode
    // config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT.in(Amps);
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT.in(Amps);
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PIDFF
    // config.Slot0.kP = ShooterConstants.kP;
    // config.Slot0.kI = ShooterConstants.kI;
    // config.Slot0.kD = ShooterConstants.kD;
    // config.Slot0.kV = ShooterConstants.kVLeft;
    // config.Slot0.kA = ShooterConstants.kALeft;
    leftMotorVoltageSignal = leftMotor.getMotorVoltage();
    leftMotorStatorCurrentSignal = leftMotor.getStatorCurrent();
    leftMotorSupplyCurrentSignal = leftMotor.getSupplyCurrent();
    leftMotorVelocitySignal = leftMotor.getVelocity();
    leftEncoderPositionSignal = leftMotor.getPosition();

    rightMotorVoltageSignal = rightMotor.getMotorVoltage();
    rightMotorStatorCurrentSignal = rightMotor.getStatorCurrent();
    rightMotorSupplyCurrentSignal = rightMotor.getSupplyCurrent();
    rightMotorVelocitySignal = rightMotor.getVelocity();
    rightEncoderPositionSignal = rightMotor.getPosition();

    addedMotorVoltageSignal = addedMotor.getMotorVoltage();
    addedMotorStatorCurrentSignal = addedMotor.getStatorCurrent();
    addedMotorSupplyCurrentSignal = addedMotor.getSupplyCurrent();
    addedMotorVelocitySignal = addedMotor.getVelocity();
    addedEncoderPositionSignal = addedMotor.getPosition();

    leftMotor.optimizeBusUtilization(Hertz.of(0));
    rightMotor.optimizeBusUtilization(Hertz.of(0));
    addedMotor.optimizeBusUtilization(Hertz.of(0));
    BaseStatusSignal.setUpdateFrequencyForAll(
      ShooterConstants.updateFrequency,
      leftMotorVoltageSignal,
      leftMotorStatorCurrentSignal,
      leftMotorSupplyCurrentSignal,
      leftMotorVelocitySignal,
      leftEncoderPositionSignal,

      rightMotorVoltageSignal,
      rightMotorStatorCurrentSignal,
      rightMotorSupplyCurrentSignal,
      rightMotorVelocitySignal,
      rightEncoderPositionSignal,

      addedMotorVoltageSignal,
      addedMotorStatorCurrentSignal,
      addedMotorSupplyCurrentSignal,
      addedMotorVelocitySignal,
      addedEncoderPositionSignal);


    //config.Slot0.kV = ShooterConstants.kVRight;
    leftMotor.getConfigurator().apply(config);
    // config.Slot0.kP = ShooterConstants.kPLeft;
    // config.Slot0.kV = ShooterConstants.kVRight;
    // config.Slot0.kA = ShooterConstants.kARight;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(config);
    addedMotor.getConfigurator().apply(config);
    //rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    //leftConfig = config.clone().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      leftMotorVoltageSignal,
      leftMotorStatorCurrentSignal,
      leftMotorSupplyCurrentSignal,
      leftMotorVelocitySignal,
      leftEncoderPositionSignal,

      rightMotorVoltageSignal,
      rightMotorStatorCurrentSignal,
      rightMotorSupplyCurrentSignal,
      rightMotorVelocitySignal,
      rightEncoderPositionSignal,

      addedMotorVoltageSignal,
      addedMotorStatorCurrentSignal,
      addedMotorSupplyCurrentSignal,
      addedMotorVelocitySignal,
      addedEncoderPositionSignal);

    inputs.leftMotorVoltage = leftMotorVoltageSignal.getValue();
    inputs.leftMotorStatorCurrent = leftMotorStatorCurrentSignal.getValue();//leftMotor.getStatorCurrent().getValue();
    inputs.leftMotorSupplyCurrent = leftMotorSupplyCurrentSignal.getValue();//leftMotor.getSupplyCurrent().getValue();
    inputs.leftMotorSpeed = leftMotorVelocitySignal.getValue();
    inputs.leftMotorConnected = leftMotor.isConnected();
    inputs.leftEncoderPosition = leftEncoderPositionSignal.getValue().in(Rotations);

    inputs.rightMotorVoltage = rightMotorVoltageSignal.getValue();
    inputs.rightMotorStatorCurrent = rightMotorStatorCurrentSignal.getValue();
    inputs.rightMotorSupplyCurrent = rightMotorSupplyCurrentSignal.getValue();
    inputs.rightMotorSpeed = rightMotorVelocitySignal.getValue();
    inputs.rightMotorConnected = rightMotor.isConnected();
    inputs.rightEncoderPosition = rightEncoderPositionSignal.getValue().in(Rotations);

    inputs.addedMotorVoltage = addedMotorVoltageSignal.getValue();
    inputs.addedMotorStatorCurrent = addedMotorStatorCurrentSignal.getValue();
    inputs.addedMotorSupplyCurrent = addedMotorSupplyCurrentSignal.getValue();
    inputs.addedMotorSpeed = addedMotorVelocitySignal.getValue();
    inputs.addedMotorConnected = addedMotor.isConnected();
    inputs.addedEncoderPosition = addedEncoderPositionSignal.getValue().in(Rotations);
  }

  @Override
  public void setShooterVelocity(AngularVelocity leftVel, AngularVelocity rightVel) {
    //leftMotor.setControl(velocityRequest.withVelocity(leftVel));
    leftMotor.setControl(velocityRequest.withVelocity(leftVel));
    //rightMotor.setControl(velocityRequest.withVelocity(0));//rightVel));
  }

  @Override
  public void setShooterVoltage(Voltage leftVolts, Voltage rightVolts) {
    leftMotor.setControl(voltageRequest.withOutput(leftVolts));
    rightMotor.setControl(voltageRequest.withOutput(rightVolts));
    addedMotor.setControl(voltageRequest.withOutput(rightVolts));
  }

  @Override
  public void stop() {
     leftMotor.setControl(voltageRequest.withOutput(0));
    rightMotor.setControl(voltageRequest.withOutput(0));
    addedMotor.setControl(voltageRequest.withOutput(0));
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    addedMotor.stopMotor();
  }

  @Override
  public void setkP(double kP){
    Slot0Configs PIDF = config.Slot0;
    PIDF.kP = kP;
    // leftMotor.getConfigurator().apply(config);
    // rightMotor.getConfigurator().apply(config);
  }
  @Override
  public void setShooterSpeeds(double speed){
    // DutyCycleOut speedRequest = new DutyCycleOut(speed);
    // speedRequest.IgnoreHardwareLimits = true;
    // speedRequest.IgnoreSoftwareLimits = true;
    leftMotor.set(speed);//setControl(speedRequest);
    rightMotor.set(speed);//setControl(speedRequest);
    addedMotor.set(speed);
  }
}
