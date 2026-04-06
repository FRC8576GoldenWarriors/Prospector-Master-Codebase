package frc.robot.subsystems.ShooterHood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterHoodIOKraken implements ShooterHoodIO {
    private final TalonFX hoodMotor;
    private final DutyCycleEncoder hoodEncoder;
    private final TalonFXConfiguration hoodConfig;

    private final StatusSignal<Voltage> shooterHoodVoltageSignal;
    private final StatusSignal<Current> shooterHoodStatorCurrentSignal;
    private final StatusSignal<Current> shooterHoodSupplyCurrentSignal;
    private final StatusSignal<AngularVelocity> shooterHoodAngularVelocitySignal;
    private final StatusSignal<Angle> shooterHoodAngleSignal;


    public ShooterHoodIOKraken() {
        hoodMotor = new TalonFX(ShooterHoodConstants.shooterHoodMotorID);
        hoodEncoder = new DutyCycleEncoder(ShooterHoodConstants.shooterHoodEncoderDIO, 1.0, ShooterHoodConstants.expectedZero);

        hoodConfig = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40))
            .withStatorCurrentLimitEnable(true)
            );

        shooterHoodVoltageSignal = hoodMotor.getMotorVoltage();
        shooterHoodStatorCurrentSignal = hoodMotor.getStatorCurrent();
        shooterHoodSupplyCurrentSignal = hoodMotor.getSupplyCurrent();
        shooterHoodAngularVelocitySignal = hoodMotor.getVelocity();
        shooterHoodAngleSignal = hoodMotor.getPosition();

        hoodMotor.optimizeBusUtilization(Hertz.of(0));

        BaseStatusSignal.setUpdateFrequencyForAll(
            ShooterHoodConstants.updateFrequency,
            shooterHoodVoltageSignal,
            shooterHoodStatorCurrentSignal,
            shooterHoodSupplyCurrentSignal,
            shooterHoodAngularVelocitySignal,
            shooterHoodAngleSignal
        );

        hoodMotor.getConfigurator().apply(hoodConfig);

        hoodEncoder.setInverted(ShooterHoodConstants.encoderInverted);
    }

    @Override
    public void updateInputs(ShooterHoodInputs inputs) {
        BaseStatusSignal.refreshAll(
            shooterHoodVoltageSignal,
            shooterHoodStatorCurrentSignal,
            shooterHoodSupplyCurrentSignal,
            shooterHoodAngularVelocitySignal,
            shooterHoodAngleSignal);

        inputs.voltage = shooterHoodVoltageSignal.getValue();
        inputs.statorCurrent = shooterHoodStatorCurrentSignal.getValue();
        inputs.supplyCurrent = shooterHoodSupplyCurrentSignal.getValue();
        inputs.speed = shooterHoodAngularVelocitySignal.getValue();
        inputs.isMotorConnected = hoodMotor.isConnected();
        //inputs.encoderValue_Radians = hoodMotor.getPosition().getValue();
        inputs.encoderValue_Radians = Rotations.of(hoodEncoder.get());
        inputs.isEncoderConnected = hoodEncoder.isConnected();
    }

    @Override
    public void setSpeed(double speed) {
        hoodMotor.set(speed);;
    }

    @Override
    public void setVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

}
