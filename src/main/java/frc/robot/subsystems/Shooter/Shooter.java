package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setWantedState(ShooterStates state) {
    this.currentState = state;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if(!DriverStation.isDisabled()) {
      switch (currentState) {
        case IDLE:
          manualRPMTarget = 0;
          io.stop();
          break;

        case SHOOT:
          wantedRPS = ShooterUtil.calculateShotVelocity(0,0);//REPLACE LATER WITH REAL PARAMETERS
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
  }
}
