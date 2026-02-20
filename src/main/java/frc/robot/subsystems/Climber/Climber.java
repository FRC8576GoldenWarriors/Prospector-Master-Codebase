// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Intake. */

  private ClimberIO io;

  public enum ClimberStates{
    Idle,
  }
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberStates wantedState = ClimberStates.Idle;
  private ProfiledPIDController PID;
  private ArmFeedforward FF;
  private double currentPosition;
  private double wantedSpeed;
  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage;
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Tuning/kP",IntakeConstants.Software.kP);
  private double kPDouble = kP.get();
  private LoggedNetworkNumber kV = new LoggedNetworkNumber("Tuning/kV",0.0);
  private double kVDouble = kV.get();

  public Climber(ClimberIO io) {
    PID = new ProfiledPIDController(ClimberConstants.Software.kP, ClimberConstants.Software.kI, ClimberConstants.Software.kD, ClimberConstants.Software.profile);
    FF = new ArmFeedforward(ClimberConstants.Software.kS, ClimberConstants.Software.kG, ClimberConstants.Software.kV, ClimberConstants.Software.kA);
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    currentPosition = inputs.leftEncoderRotations;
    kPDouble = kP.getAsDouble();

    kVDouble = kV.getAsDouble();
    PID.setP(kPDouble);
    FF.setKv(kVDouble);
     if (DriverStation.isEnabled()) {

    }else{
        wantedState = ClimberStates.Idle;
     }


    Logger.recordOutput("Intake/Wanted State", wantedState);
    Logger.recordOutput("Intake/Wanted Roller Speed", wantedSpeed);
    Logger.recordOutput("Intake/Wanted Pivot Position", PID.getSetpoint());
    Logger.recordOutput("Intake/PID Voltage", PIDVoltage);
    Logger.recordOutput("Intake/FF Voltage", FFVoltage);
    Logger.recordOutput("Intake/Input Voltage", inputVoltage);





}

public void setWantedPosition(IntakeStates wantedState) {
    this.wantedState = wantedState;
}

public void resetPID() {
    PID.reset(currentPosition, inputs.pivotRPS.magnitude());
}

// TO-DO : MAKE THIS IN MACROS CLASS
public SequentialCommandGroup setWantedState(IntakeStates state){
    return null;//new SequentialCommandGroup(new InstantCommand(()->this.resetPID(), RobotContainer.intake), new InstantCommand(()->this.setWantedPosition(state), RobotContainer.intake) );
}

//
}
