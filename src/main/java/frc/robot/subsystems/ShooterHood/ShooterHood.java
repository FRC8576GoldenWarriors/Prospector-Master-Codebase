package frc.robot.subsystems.ShooterHood;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterHood extends SubsystemBase {

    private final ShooterHoodIO io;
    private final ShooterHoodInputsAutoLogged inputs = new ShooterHoodInputsAutoLogged();
    public ProfiledPIDController PID;
    public ArmFeedforward FF;
    private double PIDVoltage;
    private double FFVoltage;
    private double inputVoltage;
    //public ShooterHoodIOKraken motor = new ShooterHoodIOKraken();
    private Alert shooterHoodMotorAlert = new Alert("The Shooter Hood Motor is disconnected", AlertType.kError);
    private Alert shooterHoodEncoderAlert = new Alert("The Shooter Hood Encoder is disconnected", AlertType.kError);


    public enum ShooterHoodStates {
        Idle,
        Rest,
        Shoot,
        HoodVoltageControl
    }

    @AutoLogOutput (key = "ShooterHood/CurrentState")
    private ShooterHoodStates currentState = ShooterHoodStates.Idle;

    private double currentAnglePosition = 0.0;
    private double wantedAnglePosition = 0.0;

    private LoggedNetworkNumber kPNumber = new LoggedNetworkNumber("Tuning/ShooterHood kP",ShooterHoodConstants.kp);
    private double kP = kPNumber.get();

    public ShooterHood(ShooterHoodIO io) {
        this.io = io;
        PID = new ProfiledPIDController(ShooterHoodConstants.kp, ShooterHoodConstants.ki, ShooterHoodConstants.kd, ShooterHoodConstants.profile);
        FF = new ArmFeedforward(ShooterHoodConstants.ks, ShooterHoodConstants.kg, ShooterHoodConstants.ks);
    }

    public void setWantedState(ShooterHoodStates state) {
    this.currentState = state;
  }

  public double getWantedPosition(double awayFromHubMeters) {
    if(awayFromHubMeters<2){
        return 40;
    }
    return 20;
  }

  @Override
  public void periodic() {
    currentAnglePosition = inputs.encoderValue.in(Rotations);
    io.updateInputs(inputs);

    kP = kPNumber.get();
    Logger.processInputs("ShooterHood", inputs);

    if(!DriverStation.isDisabled()) {
         PID.setP(kP);

        switch(currentState) {
            case Idle:
                io.setVoltage(0.0);
                break;

            case Rest:
                PIDVoltage = PID.calculate(0);
                FFVoltage = FF.calculate(0, 0);
                inputVoltage = PIDVoltage + FFVoltage;

                io.setVoltage(inputVoltage);

                break;

            case Shoot:
            wantedAnglePosition = 0.225;//getWantedPosition(1);
                PIDVoltage = PID.calculate(currentAnglePosition,wantedAnglePosition);
                FFVoltage = FF.calculate(wantedAnglePosition, 1.0);
                inputVoltage = PIDVoltage + FFVoltage;

                io.setVoltage(inputVoltage);

                break;

            case HoodVoltageControl:
                if (RobotContainer.controller.povUp().getAsBoolean()) {
                    io.setSpeed(0.05);
                } else if (RobotContainer.controller.povDown().getAsBoolean()) {
                    io.setSpeed(-0.05);
                } else {
                    setWantedState(ShooterHoodStates.Idle);
                }
                break;
    }
  } else {
        setWantedState(ShooterHoodStates.Idle);
  }
  Logger.recordOutput("ShooterHood/PID Voltage", PIDVoltage);
  Logger.recordOutput("ShooterHood/FF Voltage", FFVoltage);
    shooterHoodMotorAlert.set(!inputs.isMotorConnected);
    shooterHoodEncoderAlert.set(!inputs.isEncoderConnected);


  }
  public void resetPID(){
    PID.reset(inputs.encoderValue.in(Rotations), inputs.speed.magnitude());
  }
}
