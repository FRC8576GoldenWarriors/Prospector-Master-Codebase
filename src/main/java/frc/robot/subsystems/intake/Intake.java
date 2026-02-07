// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private IntakeIO io;

  public enum IntakeStates{
    Idle,
    Rest,
    Intake,
    PivotVC,
    RollerVC
  }
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeStates wantedState = IntakeStates.Idle;
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

  public Intake(IntakeIO io) {
    PID = new ProfiledPIDController(IntakeConstants.Software.kP, IntakeConstants.Software.kI, IntakeConstants.Software.kD, IntakeConstants.Software.profile);
    FF = new ArmFeedforward(IntakeConstants.Software.kS, IntakeConstants.Software.kG, IntakeConstants.Software.kV, IntakeConstants.Software.kA);
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    currentPosition = inputs.leftEncoderRotations;
    kPDouble = kP.getAsDouble();

    kVDouble = kV.getAsDouble();
    PID.setP(kPDouble);
    FF.setKv(kVDouble);
     if (DriverStation.isEnabled()) {
      if(currentPosition>IntakeConstants.Software.intakeSoftStop){
            wantedState = IntakeStates.Idle;
          }
      switch (wantedState) {
        case Idle:
          PIDVoltage = 0;
          FFVoltage = 0;
          inputVoltage = 0;
          wantedSpeed = 0;

          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeed(wantedSpeed);
          break;

        case Rest:
          PIDVoltage  = PID.calculate(currentPosition, IntakeConstants.Software.intakeUp);
          FFVoltage = FF.calculate(IntakeConstants.Software.intakeUp, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          wantedSpeed = 0;

          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeed(wantedSpeed);
          break;

        case Intake:
          PIDVoltage  = PID.calculate(currentPosition, IntakeConstants.Software.intakeDown);
          FFVoltage = FF.calculate(IntakeConstants.Software.intakeDown, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          wantedSpeed = IntakeConstants.Software.rollerSpeed;

          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeed(wantedSpeed);
          break;

        case PivotVC:
          if (RobotContainer.controller.povUp().getAsBoolean()) {
            io.setPivotSpeed(0.05);
            } else if (RobotContainer.controller.povDown().getAsBoolean()) {
              io.setPivotSpeed(-0.05);
            } else {
                wantedState = IntakeStates.Idle;
            }
        break;

        case RollerVC:
          if (RobotContainer.controller.rightBumper().getAsBoolean()) {
            io.setRollerSpeed(0.1);
            } else if (RobotContainer.controller.leftBumper().getAsBoolean()) {
              io.setRollerSpeed(-0.1);
            } else {
                wantedState = IntakeStates.Idle;
            }
          break;

      }
     }else{
        wantedState = IntakeStates.Idle;
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
