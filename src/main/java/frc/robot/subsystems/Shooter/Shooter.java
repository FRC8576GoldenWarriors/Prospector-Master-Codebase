package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterHood.ShooterHood;


public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private Alert leftMotorAlert = new Alert("The Left Motor is disconnected", AlertType.kError);
  private Alert rightMotorAlert = new Alert("The Right Motor is disconnected", AlertType.kError);

  public enum ShooterStates {
    IDLE,
    SHOOT,
    VOLTAGE_CONTROL_POSITIVE,
    VOLTAGE_CONTROL_NEGATIVE
  }

  private ShooterStates currentState = ShooterStates.IDLE;

  @AutoLogOutput(key = "Shooter/WantedRPS")
  private AngularVelocity wantedRPS = RotationsPerSecond.of(0);

  @AutoLogOutput(key = "Shooter/ManualRPMTarget")
  private double manualRPMTarget = 0;

  private final ShooterHood hood;

  public Shooter(ShooterIO io, ShooterHood hood) {
    this.io = io;
    this.hood = hood;
  }

  public void setWantedState(ShooterStates state) {
    this.currentState = state;
  }


  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    //Establishing and updating value of RPM
    manualRPMTarget = SmartDashboard.getNumber("ManualRPMTarget: ", manualRPMTarget);
    SmartDashboard.putNumber("Enter Value for ManualRPMTarget: ", manualRPMTarget);

    //For now we are using manual shooter to hub distance till we get limelight data
    double manualDistanceMeters = SmartDashboard.getNumber("Shooter to Hub ManualDistanceMeters: ", 0.0);
    SmartDashboard.putNumber("ManualDistanceMeters", manualDistanceMeters);

    //SmartDashboard Outputs
    SmartDashboard.putNumber("Shooter/LeftRPM", inputs.leftMotorSpeed);
    SmartDashboard.putNumber("Shooter/RightRPM", inputs.rightMotorSpeede);

    if(!DriverStation.isDisabled()) {
      switch (currentState) {
        case IDLE:
          manualRPMTarget = 0;
          io.stop();
          break;

        case SHOOT:
          wantedRPS = ShooterUtil.calculateShotVelocity(manualDistanceMeters, hood.getCurrentAnglePosition());//REPLACE LATER WITH REAL PARAMETER for distance
          io.setShooterVelocity(wantedRPS, wantedRPS);
          break;

        case VOLTAGE_CONTROL_POSITIVE:
          manualRPMTarget += ShooterConstants.MANUAL_STEP_RPM;
          manualRPMTarget = Math.min(manualRPMTarget, 6000);
          io.setShooterVelocity(RotationsPerSecond.of(manualRPMTarget / 60.0),
                                RotationsPerSecond.of(manualRPMTarget / 60.0));
          break;

        case VOLTAGE_CONTROL_NEGATIVE:
          manualRPMTarget -= ShooterConstants.MANUAL_STEP_RPM;
          manualRPMTarget = Math.max(manualRPMTarget, 0);
          io.setShooterVelocity(RotationsPerSecond.of(manualRPMTarget / 60.0),
                                RotationsPerSecond.of(manualRPMTarget / 60.0));
          break;
      }
    } else {
      currentState = ShooterStates.IDLE;
    }

    leftMotorAlert.set(!inputs.leftMotorConnected);
    rightMotorAlert.set(!inputs.rightMotorConnected);


  }
}
