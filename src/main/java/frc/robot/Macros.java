// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHood.ShooterHoodStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportStates;
import frc.robot.subsystems.vision.VisionConstants;


@SuppressWarnings("unused")
public class Macros extends SubsystemBase {
  private Shooter m_shooter;
  private ShooterHood m_ShooterHood;
  private Transport m_Transport;
  private Intake m_Intake;

    //TO-DO: Add climber when comp bot
  public enum RobotStates{
    Idle,
    Shoot,
    IntakeOn,
    ClimbUp,
    ClimbDown,
    IntakeOff,
    Rest,
    RunContinous,
    TransportOut,
    TransportIn,
    AutonShoot
    //Testing
  }


  @AutoLogOutput (key="Robot/Direct State Log")
  private RobotStates wantedState = RobotStates.Idle;

  /** Creates a new Macros. */
  public Macros(Shooter shooter, ShooterHood hood, Transport transport, Intake intake) {
    m_ShooterHood = hood;
    m_shooter = shooter;
    m_Transport = transport;
    m_Intake = intake;
  }


  @Override
  public void periodic() {
    if(DriverStation.isEnabled()||DriverStation.isAutonomous()){
     switch(wantedState){
        case Idle:
            Idle();
            break;
        case Shoot:
            shoot();
            break;
        case IntakeOn:
            intakeOn();
            break;
        case IntakeOff:
            intakeOff();
            break;
        case Rest:
            rest();
            break;
        case RunContinous:
          runContinous();
          break;
        case TransportOut:
          transportOut();
          break;
        case TransportIn:
          transportIn();
          break;
        case AutonShoot:
          autonShoot();
        default:
            break;
     }
  }
  else{
    wantedState = RobotStates.Idle;
  }
    Logger.recordOutput("Robot/Wanted State", wantedState);
  }





  public void setWantedStatePrivate(RobotStates s){
    wantedState = s;
    Logger.recordOutput("Robot/Private Wanted State", s);
  }
  public void Idle(){
    m_shooter.setWantedState(ShooterStates.IDLE);
    m_ShooterHood.setWantedState(ShooterHoodStates.Idle);
    m_Transport.setWantedState(TransportStates.Idle);
    m_Intake.setWantedPosition(IntakeStates.Idle);
  }
  public void rest(){
    m_shooter.setWantedState(ShooterStates.IDLE);
    m_ShooterHood.setWantedState(ShooterHoodStates.Idle);
    m_Transport.setWantedState(TransportStates.Idle);
    m_Intake.setWantedPosition(IntakeStates.Rest);
  }
  public void intakeOn(){
    if(!m_Intake.nearSetpoint()){
    m_Intake.setWantedPosition(IntakeStates.IntakeDown);
    }else{
      m_Intake.setWantedPosition(IntakeStates.Intake);
    }
  }
  public void transportOut(){
    // m_shooter.setWantedState(ShooterStates.IDLE);
    // m_ShooterHood.setWantedState(ShooterHoodStates.Idle);
    m_Transport.setWantedState(TransportStates.TransportOut);
    //m_Intake.setWantedPosition(IntakeStates.Idle);
  }
  public void transportIn(){
    m_Transport.setWantedState(TransportStates.TransportIn);
  }
  public void intakeOff(){
    m_Intake.setWantedPosition(IntakeStates.Rest);
  }
  public void shoot(){
    if(RobotContainer.driveController.rightTrigger().getAsBoolean()){
      if((RobotContainer.drive.getPose().getX()  > (VisionConstants.aprilTagLayout.getFieldLength()-4.61)&&DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red) ||(RobotContainer.drive.getPose().getX() < 4.61&&DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue)){
        if(m_ShooterHood.atSetpoint()){
      m_shooter.setWantedState(ShooterStates.SHOOT);
    }
    if(m_shooter.isRevved()&&m_ShooterHood.atSetpoint()){
    //m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
    m_Transport.setWantedState(TransportStates.TransportIn);
     }
     //else{
    //   m_Transport.setWantedState(TransportStates.Idle);
    // }
    if(m_Intake.getState()==IntakeStates.Idle){
   m_Intake.setWantedPosition(IntakeStates.IntakeDown);
   }
    if(!(m_Intake.getState()==IntakeStates.Agitate)&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.Agitate);
    }
    else if(m_Intake.getState()==IntakeStates.Agitate&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.IntakeDown);
    }

    m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
  //   if(m_ShooterHood.atSetpoint()){
  //     m_shooter.setWantedState(ShooterStates.SHOOT);
  //   }
  //   if(m_shooter.isRevved()&&m_ShooterHood.atSetpoint()){
  //   //m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
  //   m_Transport.setWantedState(TransportStates.TransportIn);
  //   }else{
  //     m_Transport.setWantedState(TransportStates.Idle);
  //   }
  //   if(m_Intake.getState()==IntakeStates.Idle){
  //  m_Intake.setWantedPosition(IntakeStates.IntakeDown);
  //  }
  //   if(!(m_Intake.getState()==IntakeStates.Agitate)&&m_Intake.nearSetpoint()){
  //     m_Intake.setWantedPosition(IntakeStates.Agitate);
  //   }
  //   else if(m_Intake.getState()==IntakeStates.Agitate&&m_Intake.nearSetpoint()){
  //     m_Intake.setWantedPosition(IntakeStates.IntakeDown);
  //   }

  //   m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
  }else{
    if(m_ShooterHood.atSetpoint()){
      m_shooter.setWantedState(ShooterStates.SHOOT);
    }
    if(m_shooter.isRevved()&&m_ShooterHood.atSetpoint()){
    //m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
    m_Transport.setWantedState(TransportStates.TransportIn);
    }else{
      m_Transport.setWantedState(TransportStates.Idle);
    }
    if(m_Intake.getState()==IntakeStates.Idle){
   m_Intake.setWantedPosition(IntakeStates.IntakeDown);
   }
    if(!(m_Intake.getState()==IntakeStates.Agitate)&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.Agitate);
    }
    else if(m_Intake.getState()==IntakeStates.Agitate&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.IntakeDown);
    }

    m_ShooterHood.setWantedState(ShooterHoodStates.Passing);
  }

  }else{
    wantedState = RobotStates.Idle;
  }
   // m_Intake.setWantedPosition(IntakeStates.Intake);
  }
  public void autonShoot(){
    if(m_ShooterHood.atSetpoint()){
      m_shooter.setWantedState(ShooterStates.SHOOT);
    }
    if(m_shooter.isRevved()&&m_ShooterHood.atSetpoint()){
    //m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
    m_Transport.setWantedState(TransportStates.TransportIn);
     }
     //else{
    //   m_Transport.setWantedState(TransportStates.Idle);
    // }
    if(m_Intake.getState()==IntakeStates.Idle){
   m_Intake.setWantedPosition(IntakeStates.IntakeDown);
   }
    if(!(m_Intake.getState()==IntakeStates.Agitate)&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.Agitate);
    }
    else if(m_Intake.getState()==IntakeStates.Agitate&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.IntakeDown);
    }

    m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
  }

  public void runContinous(){
    if(m_ShooterHood.atSetpoint()){
      m_shooter.setWantedState(ShooterStates.Tuning);
    }
    if(m_shooter.isRevved()&&m_ShooterHood.atSetpoint()){
    //m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
    m_Transport.setWantedState(TransportStates.TransportIn);
     }
     //else{
    //   m_Transport.setWantedState(TransportStates.Idle);
    // }
    if(m_Intake.getState()==IntakeStates.Idle){
   m_Intake.setWantedPosition(IntakeStates.IntakeDown);
   }
    if(!(m_Intake.getState()==IntakeStates.Agitate)&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.Agitate);
    }
    else if(m_Intake.getState()==IntakeStates.Agitate&&m_Intake.nearSetpoint()){
      m_Intake.setWantedPosition(IntakeStates.IntakeDown);
    }

    m_ShooterHood.setWantedState(ShooterHoodStates.Test);
  //  m_shooter.setWantedState(ShooterStates.Tuning);
    //m_ShooterHood.setWantedState(ShooterHoodStates.Test);
  //  if(m_Intake.getState()==IntakeStates.Idle){
  //  m_Intake.setWantedPosition(IntakeStates.IntakeDown);
  //  }
    //if(m_shooter.isRevved()&&m_ShooterHood.atSetpoint()){
    //m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
    // m_Transport.setWantedState(TransportStates.TransportIn);
    //m_shooter.setWantedState(ShooterStates.Tuning);
    // }else{
    //   m_Transport.setWantedState(TransportStates.Idle);
    // }
    //m_Transport.setWantedState(TransportStates.TransportIn);
    // if(!(m_Intake.getState()==IntakeStates.Agitate)&&m_Intake.nearSetpoint()){
    //   m_Intake.setWantedPosition(IntakeStates.Agitate);
    // }
    // else if(m_Intake.getState()==IntakeStates.Agitate&&m_Intake.nearSetpoint()){
    //   m_Intake.setWantedPosition(IntakeStates.IntakeDown);
    // }
    //m_Transport.setWantedState(TransportStates.TransportIn);
  }
  public SequentialCommandGroup setWantedState(RobotStates wantedState){
    return new SequentialCommandGroup(
        Commands.parallel(new InstantCommand(()->m_Intake.resetPID(), m_Intake)
        ), new InstantCommand(()->this.setWantedStatePrivate(wantedState), this));
    }




}
