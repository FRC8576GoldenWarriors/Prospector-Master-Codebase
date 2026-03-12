package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private SysIdRoutine sysId;
  private Alert leftMotorAlert = new Alert("The Left Motor is disconnected", AlertType.kError);
  private Alert rightMotorAlert = new Alert("The Right Motor is disconnected", AlertType.kError);

  private LoggedNetworkNumber kPNumber = new LoggedNetworkNumber("Tuning/Shooter kP",ShooterConstants.kP);
  private SimpleMotorFeedforward ff;
  private double pastkP = kPNumber.get();
  private double currentkP = kPNumber.get();
    private LoggedNetworkNumber targetRPS = new LoggedNetworkNumber("Tuning/Shooter Target RPS",35);
  private BangBangController bangBangController = new BangBangController(0.05/2);
  private double target = targetRPS.get();
  public enum ShooterStates {
    IDLE,
    SHOOT,
    Tuning,
    VOLTAGE_CONTROL_POSITIVE,
    VOLTAGE_CONTROL_NEGATIVE,
    SYSID
  }

  private ShooterStates currentState = ShooterStates.IDLE;

  @AutoLogOutput(key = "Shooter/WantedRPS")
  private AngularVelocity wantedRPS = RotationsPerSecond.of(0);

  @AutoLogOutput(key = "Shooter/ManualRPMTarget")
  private double manualRPMTarget = 0;

  public Shooter(ShooterIO io) {
    this.io = io;
    sysId = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, null, null,
      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      (voltage) -> io.setShooterVoltage(voltage, voltage),
      null, // No log consumer, since data is recorded by AdvantageKit
      this
    )
  );
  }

  public void setWantedState(ShooterStates state) {
    this.currentState = state;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/WantedState", currentState);
    currentkP = kPNumber.get();
    wantedRPS = RotationsPerSecond.of(targetRPS.get());//(ShooterUtil.calculateShotVelocity(RobotContainer.drive.getDistanceFromHub(),(Units.rotationsToDegrees(RobotContainer.shooterHood.getAngle())+22)/4)).times((1.15));//
    Logger.recordOutput("Shooter/leftMotorSpeedErrorRPS", Math.abs(inputs.leftMotorSpeed.in(RotationsPerSecond) - wantedRPS.in(RotationsPerSecond)));
    Logger.recordOutput("Shooter/rightMotorSpeedErrorRPS", Math.abs(inputs.rightMotorSpeed.in(RotationsPerSecond) - wantedRPS.in(RotationsPerSecond)));


      if(DriverStation.isDisabled()) {
          currentState = ShooterStates.IDLE;
          }else{
            if(currentkP!=pastkP){
              io.setkP(currentkP);
              pastkP = currentkP;
            }
                //io.setkP(kP);
      switch (currentState) {
        case IDLE:
          manualRPMTarget = 0;
          io.stop();

          break;

        case SHOOT:
          wantedRPS = RobotContainer.shooterUtil.getRPS(RobotContainer.drive.getDistanceFromHub());//(ShooterUtil.calculateShotVelocity(RobotContainer.drive.getDistanceFromHub(),(Units.rotationsToDegrees(RobotContainer.shooterHood.getAngle())+22)/4)).plus(RotationsPerSecond.of(15));//RotationsPerSecond.of(30);//RotationsPerSecond.of(target);//ShooterUtil.calculateShotVelocity(0,0);//REPLACE LATER WITH REAL PARAMETERS
          setShooter(wantedRPS);
          break;
        case Tuning:
          wantedRPS = RotationsPerSecond.of(50);//50);
          setShooter(wantedRPS);

        case VOLTAGE_CONTROL_POSITIVE:
          manualRPMTarget += ShooterConstants.MANUAL_STEP_RPM;
          manualRPMTarget = 1000;//Math.min(manualRPMTarget, 6000);
          io.setShooterVelocity(RPM.of(manualRPMTarget),
                                RPM.of(manualRPMTarget));
          break;

        case VOLTAGE_CONTROL_NEGATIVE:
          manualRPMTarget -= ShooterConstants.MANUAL_STEP_RPM;
          manualRPMTarget = Math.max(manualRPMTarget, 0);
          io.setShooterVelocity(RPM.of(manualRPMTarget),
                                RPM.of(manualRPMTarget));
          break;
        case SYSID:
        break;
      }
      Logger.recordOutput("Shooter/Wanted Speed", wantedRPS);
    }

    leftMotorAlert.set(!inputs.leftMotorConnected);
    rightMotorAlert.set(!inputs.rightMotorConnected);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @AutoLogOutput(key = "Shooter/IsRevved")
  public boolean isRevved(){
    return MathUtil.isNear(wantedRPS.in(RotationsPerSecond),(inputs.leftMotorSpeed.in(RotationsPerSecond)+inputs.rightMotorSpeed.in(RotationsPerSecond))/2,4);//1
    //return (inputs.leftMotorSpeed.in(RotationsPerSecond)>targetRPS.get()-10)&&(inputs.leftMotorSpeed.in(RotationsPerSecond)<targetRPS.get()+10);//0.5
  }

   public void setShooter(AngularVelocity wantedRPS){
    double bangBangCalculation = bangBangController.calculate(inputs.leftMotorSpeed.in(RotationsPerSecond),wantedRPS.in(RotationsPerSecond));
    // if(bangBangCalculation==0){
    //   io.setShooterVelocity(wantedRPS, wantedRPS);
    // }else{
      io.setShooterSpeeds(bangBangCalculation);
    //}
    Logger.recordOutput("Shooter/BB Controller", bangBangCalculation);
   }
}
