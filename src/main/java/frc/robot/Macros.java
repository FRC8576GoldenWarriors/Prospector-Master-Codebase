// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterStates;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHood.ShooterHoodStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportStates;


@SuppressWarnings("unused")
public class Macros extends SubsystemBase {
  private Shooter m_shooter;
  private ShooterHood m_ShooterHood;
  private Transport m_Transport;
  private Intake m_Intake;
  
    //TO-DO: Add climber when comp bot
  public enum states{
    Idle,
    Shoot,
    IntakeOn,
    ClimbUp,
    ClimbDown,
    IntakeOff,
  }

  
  private states wantedState = states.Idle;
  
  /** Creates a new Macros. */
  public Macros(Shooter shooter, ShooterHood hood, Transport transport, Intake intake) {
    m_ShooterHood = hood;
    m_shooter = shooter;
    m_Transport = transport;
    m_Intake = intake;
  }


  @Override
  public void periodic() {
    if(DriverStation.isEnabled()){
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
        default:
            break;
     }
  }
  else{
    wantedState = states.Idle;
  }
    Logger.recordOutput("Robot/Wanted State", wantedState);
  }
  
  
  
  

  public void setWantedState(states wantedState){
    this.wantedState = wantedState;
  }
  public void Idle(){
    m_shooter.setWantedState(ShooterStates.IDLE);
    m_ShooterHood.setWantedState(ShooterHoodStates.Idle);
    m_Transport.setWantedState(TransportStates.Idle);
    m_Intake.setWantedState(IntakeStates.Rest);
  }
  public void intakeOn(){
    m_Intake.setWantedState(IntakeStates.Intake);
  }
  public void intakeOff(){
    m_Intake.setWantedState(IntakeStates.Rest);
  }
  public void shoot(){
    m_shooter.setWantedState(ShooterStates.SHOOT);
    m_ShooterHood.setWantedState(ShooterHoodStates.Shoot);
    m_Transport.setWantedState(TransportStates.TransportIn);
  }
  


}
