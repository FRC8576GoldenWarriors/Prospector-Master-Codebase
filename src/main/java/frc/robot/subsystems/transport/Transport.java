// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transport extends SubsystemBase {

  private final TransportIO io;
  private final TransportIOInputsAutoLogged inputs = new TransportIOInputsAutoLogged();


  @AutoLogOutput(key = "Transport/CurrentState")
  private TransportStates currentState = TransportStates.Idle;

  public enum TransportStates {
    Idle,
    TransportIn,
    TransportOut,
    Rest
  }

  public Transport(TransportIO io) {
    this.io = io;
  }

  public void setWantedState(TransportStates state) {
    this.currentState = state;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transport", inputs);

    if (DriverStation.isDisabled()){
      // !RobotContainer.controller.rightBumper().getAsBoolean() ||
      // (RobotContainer.controller.rightBumper().getAsBoolean() && !isFuelDetected())) {
      currentState = TransportStates.Idle;
    }
    else{
    switch (currentState) {
      case Idle:
        io.setTransportVoltage(0);
        break;

      case TransportIn:
        io.setTransportSpeed(RotationsPerSecond.of(TransportConstants.velocityTarget),RotationsPerSecond.of(TransportConstants.velocityTarget));
        break;

      case TransportOut:
        io.setTransportSpeed(RotationsPerSecond.of(-TransportConstants.velocityTarget),RotationsPerSecond.of(-TransportConstants.velocityTarget));
        break;

      case Rest:
        io.setTransportSpeed(RotationsPerSecond.of(TransportConstants.velocityTargetSlower),RotationsPerSecond.of(TransportConstants.velocityTargetSlower));
        break;
      default:
        io.setTransportSpeed(RPM.of(0),RPM.of(0));
        currentState = TransportStates.Idle;
        break;
    }
   }
  }

  @AutoLogOutput (key = "Transport/No Fuel Detected")
  public boolean noFuelDetected() {
    return !hasFuel();
  }

  @AutoLogOutput (key = "Transport/Has Fuel")
  public boolean hasFuel() {
    return inputs.leftFuelDetected || inputs.rightFuelDetected;
  }
}
