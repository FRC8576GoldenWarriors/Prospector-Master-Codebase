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
    TransportOut
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

    switch (currentState) {
      case Idle:
        io.setTransportSpeed(RPM.of(0));
        break;

      case TransportIn:
        io.setTransportSpeed(RotationsPerSecond.of(75));;
        break;

      case TransportOut:
        io.setTransportSpeed(RotationsPerSecond.of(-75));;
        break;

      default:
        io.setTransportSpeed(RPM.of(0));
        currentState = TransportStates.Idle;
        break;
    }
  }

  public boolean isFuelDetected() {
    return inputs.leftFuelDetected || inputs.rightFuelDetected;
  }
}
