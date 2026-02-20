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
import frc.robot.RobotContainer;


public class Climber extends SubsystemBase {
  /** Creates a new Intake. */

  private ClimberIO io;

  public enum ClimberStates{
    Idle,
    AutonClimb,
    TeleClimb,
    VoltageControl
  }
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberStates wantedState = ClimberStates.Idle;
  private ProfiledPIDController PID;
  private ArmFeedforward FF;
  private double currentPosition;
  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage;
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Tuning/kP",ClimberConstants.Software.kP);
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
    currentPosition = inputs.encoderRotations;
    kPDouble = kP.getAsDouble();

    kVDouble = kV.getAsDouble();
    PID.setP(kPDouble);
    FF.setKv(kVDouble);
     if (DriverStation.isEnabled()) {
      if(inputs.photoelectricDetected){
            wantedState = ClimberStates.Idle;
        }
      switch (wantedState) {
        case Idle:
          PIDVoltage  = 0;
          FFVoltage   = 0;
          inputVoltage= 0;
          break;
        case AutonClimb:
          if(inputs.photoelectricDetected){
            break;
          }

          PIDVoltage  = PID.calculate(currentPosition, ClimberConstants.Software.autoClimb);
          FFVoltage = FF.calculate(ClimberConstants.Software.autoClimb, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;

          io.setPivotSpeed(inputVoltage);
          break;

        case TeleClimb:
          if(inputs.photoelectricDetected){
            break;
          }

          PIDVoltage  = PID.calculate(currentPosition, ClimberConstants.Software.teleClimb);
          FFVoltage = FF.calculate(ClimberConstants.Software.teleClimb, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;

          io.setPivotSpeed(inputVoltage);
          break;

        case VoltageControl:
          if (RobotContainer.controller.rightBumper().getAsBoolean()) {
            io.setPivotSpeed(0.1);
            } else if (RobotContainer.controller.leftBumper().getAsBoolean()) {
              io.setPivotSpeed(-0.1);
            } else {
                wantedState = ClimberStates.Idle;
            }
          break;



        default:
          break;
      }
    }else{
        wantedState = ClimberStates.Idle;
     }


    Logger.recordOutput("Climber/Wanted State", wantedState);
    Logger.recordOutput("Climber/Wanted Pivot Position", PID.getSetpoint());
    Logger.recordOutput("Climber/PID Voltage", PIDVoltage);
    Logger.recordOutput("Climber/FF Voltage", FFVoltage);
    Logger.recordOutput("Climber/Input Voltage", inputVoltage);





}

public double findDiff(){
  return inputs.frontLeftCANRangeDistance - inputs.frontRightANRangeDistance;
}

public void setWantedPosition(ClimberStates wantedState) {
    this.wantedState = wantedState;
}

public void resetPID() {
    PID.reset(currentPosition, inputs.pivotRPS.magnitude());
}

// TO-DO : MAKE THIS IN MACROS CLASS
public SequentialCommandGroup setWantedState(ClimberStates state){
    return null;//new SequentialCommandGroup(new InstantCommand(()->this.resetPID(), RobotContainer.intake), new InstantCommand(()->this.setWantedPosition(state), RobotContainer.intake) );
}


}
