package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import org.littletonrobotics.junction.Logger;

public class ShooterHoodSubsystem extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(40);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(3);

    private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private static final double kFF = 0.0;
    private static final double REST_ANGLE_ROTATIONS = 0.15;

    public enum HoodState {
        IDLE,
        REST,
        SHOOT,
        VOLTAGE_CONTROL
    }

    private HoodState currentState = HoodState.IDLE;

    private double wantedAngleRotations = 0.0;
    private double manualVoltage = 0.0;
    private double shootDistanceMeters = 0.0;

    public ShooterHoodSubsystem() {
        pid.setTolerance(0.005);
    }

    public void setIdle() {
        currentState = HoodState.IDLE;
    }

    public void setRest() {
        currentState = HoodState.REST;
        wantedAngleRotations = REST_ANGLE_ROTATIONS;
    }

    public void setShoot(double distanceMeters) {
        currentState = HoodState.SHOOT;
        shootDistanceMeters = distanceMeters;
    }

    public void setVoltageControl(double voltage) {
        currentState = HoodState.VOLTAGE_CONTROL;
        manualVoltage = voltage;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case IDLE:
                hoodMotor.setControl(new VoltageOut(0.0));
                break;

            case REST:
                runPositionControl(wantedAngleRotations);
                break;

            case SHOOT:
                wantedAngleRotations = calculateHoodAngle(shootDistanceMeters);
                runPositionControl(wantedAngleRotations);
                break;

            case VOLTAGE_CONTROL:
                hoodMotor.setControl(new VoltageOut(manualVoltage));
                break;
        }

        logOutputs();
    }

    private void runPositionControl(double targetRotations) {
        double currentRotations = getHoodPositionRotations();
        double output = pid.calculate(currentRotations, targetRotations) + kFF;
        hoodMotor.setControl(new VoltageOut(output));
    }

    private double getHoodPositionRotations() {
        return hoodEncoder.getAbsolutePosition();
    }

    private double calculateHoodAngle(double distanceMeters) {
        double a = 0.0;
        double b = 0.0;
        double fudgeFactor = 0.0;
        return a * distanceMeters + b + fudgeFactor;
    }

    private void logOutputs() {
        Logger.recordOutput("Hood/State", currentState.toString());
        Logger.recordOutput("Hood/WantedAngleRotations", wantedAngleRotations);
        Logger.recordOutput("Hood/EncoderRotations", getHoodPositionRotations());
        Logger.recordOutput("Hood/EncoderConnected", hoodEncoder.isConnected());
        Logger.recordOutput("Hood/MotorVoltage", hoodMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Hood/MotorStatorCurrent", hoodMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Hood/MotorSupplyCurrent", hoodMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Hood/MotorSpeedRPS", hoodMotor.getVelocity().getValue());
        Logger.recordOutput("Hood/MotorConnected", hoodMotor.isConnected());
    }
}
