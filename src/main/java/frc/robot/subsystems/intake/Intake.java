// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
    IntakeDown,
    Agitate,
    PivotVC,
    RollerVC,
    IntakeOut
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
  private double pastkP = kP.get();
  private LoggedNetworkNumber kG = new LoggedNetworkNumber("Tuning/kG",IntakeConstants.Software.kG);
  private double kGDouble = kG.get();
    private double pastkG = kG.get();

  Debouncer debounce = new Debouncer(0.1,DebounceType.kRising);

  private Alert pivotMotorAlert = new Alert("The Pivot Motor is disconnected", AlertType.kError);
  private Alert rollerMotorAlert = new Alert("The Roller Motor is disconnected", AlertType.kError);
  private Alert leftEncoderAlert = new Alert("The Left Encoder is disconnected", AlertType.kError);
  private Alert rightEncoderAlert = new Alert("The Right Encoder is disconnected", AlertType.kError);

  public Intake(IntakeIO io) {
    PID = new ProfiledPIDController(IntakeConstants.Software.kP, IntakeConstants.Software.kI, IntakeConstants.Software.kD, IntakeConstants.Software.profile);
    FF = new ArmFeedforward(IntakeConstants.Software.kS, IntakeConstants.Software.kG, IntakeConstants.Software.kV, IntakeConstants.Software.kA);
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    currentPosition = inputs.rightEncoderRotations;
    kPDouble = kP.getAsDouble();
    kGDouble = kG.getAsDouble();
    if(kPDouble!=pastkP){
              PID.setP(kPDouble);
              pastkP = kPDouble;
            }
            if(kGDouble!=pastkG){
              FF.setKg(kGDouble);
              pastkG = kGDouble;
            }

     if (DriverStation.isEnabled()) {
      // if(currentPosition>IntakeConstants.Software.intakeSoftStop){
      //       wantedState = IntakeStates.Idle;
      //     }
        //PID.setP(kPDouble);
        //FF.setKg(kVDouble);
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
          PIDVoltage  = PID.calculate(currentPosition, IntakeConstants.Software.intakeUp);//intakeUp
          FFVoltage = FF.calculate(IntakeConstants.Software.intakeUp, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          wantedSpeed = 0;

          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeed(wantedSpeed);
          break;
        case IntakeDown:
          PIDVoltage  = PID.calculate(currentPosition, IntakeConstants.Software.intakeDown);
          FFVoltage = FF.calculate(IntakeConstants.Software.intakeDown, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          wantedSpeed = 0;

          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeed(wantedSpeed);
          break;
        case Agitate:
          PIDVoltage  = PID.calculate(currentPosition, IntakeConstants.Software.agitatePosition);
          FFVoltage = FF.calculate(IntakeConstants.Software.agitatePosition, 0.5);
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
          if (RobotContainer.driveController.povUp().getAsBoolean()) {
            io.setPivotSpeed(0.05);
            } else if (RobotContainer.driveController.povDown().getAsBoolean()) {
              io.setPivotSpeed(-0.05);
            } else {
                wantedState = IntakeStates.Idle;
            }
        break;

        case RollerVC:
          if (RobotContainer.driveController.rightBumper().getAsBoolean()) {
            io.setRollerSpeed(0.1);
            } else if (RobotContainer.driveController.leftBumper().getAsBoolean()) {
              io.setRollerSpeed(-0.1);
            } else {
                wantedState = IntakeStates.Idle;
            }
          break;
        case IntakeOut:
          PIDVoltage  = PID.calculate(currentPosition, IntakeConstants.Software.intakeDown);
          FFVoltage = FF.calculate(IntakeConstants.Software.intakeDown, 0.5);
          inputVoltage = PIDVoltage + FFVoltage;
          wantedSpeed = IntakeConstants.Software.rollerSpeed*(-0.75);

          io.setPivotVoltage(inputVoltage);
          io.setRollerSpeed(wantedSpeed);
          break;
      }
     }else{
        wantedState = IntakeStates.Idle;
     }

      pivotMotorAlert.set(!inputs.pivotConnected);
      rollerMotorAlert.set(!inputs.rollerConnected);
      leftEncoderAlert.set(!inputs.leftEncoderConnected);
      rightEncoderAlert.set(!inputs.rightEncoderConnected);
      Logger.recordOutput("Intake/Wanted State", wantedState);
      Logger.recordOutput("Intake/PID", PIDVoltage);
      Logger.recordOutput("Intake/FF", FFVoltage);
      Logger.recordOutput("Intake/Input Voltage", inputVoltage);
}

public void setWantedPosition(IntakeStates wantedState) {
    this.wantedState = wantedState;
}

public void resetPID() {
    PID.reset(currentPosition, 0);
}
@AutoLogOutput (key = "Intake/Near Setpoint")
public boolean nearSetpoint(){
  return MathUtil.isNear(PID.getGoal().position, inputs.rightEncoderRotations, 0.12)||debounce.calculate(inputs.pivotSupplyCurrent.in(Amps)>=9);
}
public boolean nearSetpointAgitate(){
  return MathUtil.isNear(PID.getGoal().position, inputs.rightEncoderRotations, 0.1)||(inputs.pivotSupplyCurrent.in(Amps)>10);
}
public IntakeStates getState(){
  return wantedState;
}

// TO-DO : MAKE THIS IN MACROS CLASS
public SequentialCommandGroup setWantedState(IntakeStates state){
    return null;//new SequentialCommandGroup(new InstantCommand(()->this.resetPID(), RobotContainer.intake), new InstantCommand(()->this.setWantedPosition(state), RobotContainer.intake) );
}

//
}
