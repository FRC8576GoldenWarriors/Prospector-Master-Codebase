// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX motor;
  final TalonFXConfiguration config = new TalonFXConfiguration().withMotorOutput(
    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast)
  );
  LoggedNetworkNumber kP = new LoggedNetworkNumber("Tuning/kP",1);
  LoggedNetworkNumber rps = new LoggedNetworkNumber("Tuning/Rps",40);
  double rpsDouble = rps.getAsDouble();
  final VelocityVoltage voltageRequest = new VelocityVoltage(0).withSlot(0);
  final Slot0Configs pidConfigs = new Slot0Configs();
  private double maxRPS = 7700/60.0;
    public Shooter(int motorID) {
    motor = new TalonFX(motorID);
    motor.getConfigurator().apply(config);
    pidConfigs.kV = (12)/maxRPS;
    pidConfigs.kP = kP.getAsDouble();
    motor.getConfigurator().apply(pidConfigs);
  }

  public double getSpeed() {
    return motor.get();
  }

  public void setRPS(double rps){
    if(rps>0){
    motor.setControl(voltageRequest.withVelocity(RotationsPerSecond.of(rpsDouble)));
    }else{
      motor.setControl(voltageRequest.withVelocity(RotationsPerSecond.of(0)));
    }
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidConfigs.kP = kP.getAsDouble();
    rpsDouble = rps.getAsDouble();
    motor.getConfigurator().apply(pidConfigs);
    Logger.recordOutput("getStatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("getSupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("getTorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput("getMotorStallCurrent", motor.getMotorStallCurrent().getValueAsDouble());
    Logger.recordOutput("getMotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("acceleration", motor.getAcceleration().getValueAsDouble());
  }
}
