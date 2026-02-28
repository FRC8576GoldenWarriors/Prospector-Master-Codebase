package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private TalonFXConfiguration config;
    private TalonFXConfiguration leftConfig;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ShooterIOReal() {
    leftMotor = new TalonFX(ShooterConstants.LEFT_SHOOTER_ID);
    rightMotor = new TalonFX(ShooterConstants.RIGHT_SHOOTER_ID);

    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Current Limits & Brake Mode
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PIDFF
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kS = ShooterConstants.kS;



    config.Slot0.kV = ShooterConstants.kVRight;
    rightMotor.getConfigurator().apply(config);
    config.Slot0.kP = ShooterConstants.kPLeft;
    leftMotor.getConfigurator().apply(config);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    //leftConfig = config.clone().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));


  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    StatusSignal<AngularVelocity> leftStatus = leftMotor.getVelocity();
    StatusSignal<AngularVelocity> rightStatus = rightMotor.getVelocity();

    inputs.leftMotorVoltage = leftMotor.getMotorVoltage().getValue();
    inputs.leftMotorStatorCurrent = leftMotor.getStatorCurrent().getValue();
    inputs.leftMotorSupplyCurrent = leftMotor.getSupplyCurrent().getValue();
    inputs.leftMotorSpeed = leftStatus.getValue();
    inputs.leftMotorConnected = leftMotor.isConnected();

    inputs.rightMotorVoltage = rightMotor.getMotorVoltage().getValue();
    inputs.rightMotorStatorCurrent = rightMotor.getStatorCurrent().getValue();
    inputs.rightMotorSupplyCurrent = rightMotor.getSupplyCurrent().getValue();
    inputs.rightMotorSpeed = rightStatus.getValue();
    inputs.rightMotorConnected = rightMotor.isConnected();
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
  }

  @Override
  public void stop() {
     leftMotor.setControl(voltageRequest.withOutput(0));
    rightMotor.setControl(voltageRequest.withOutput(0));
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void setkP(double kP){
    Slot0Configs PIDF = config.Slot0;
    PIDF.kP = kP;
    //leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
  }
}
