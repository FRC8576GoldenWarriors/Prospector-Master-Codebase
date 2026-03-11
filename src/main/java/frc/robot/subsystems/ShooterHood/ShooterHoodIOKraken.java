package frc.robot.subsystems.ShooterHood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterHoodIOKraken implements ShooterHoodIO {
    public TalonFX hoodMotor;
    public DutyCycleEncoder hoodEncoder;
    TalonFXConfiguration hoodConfig;


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

        hoodMotor.getConfigurator().apply(hoodConfig);

        hoodEncoder.setInverted(ShooterHoodConstants.encoderInverted);
    }

    @Override
    public void updateInputs(ShooterHoodInputs inputs) {
        inputs.voltage = hoodMotor.getMotorVoltage().getValue();
        inputs.statorCurrent = hoodMotor.getStatorCurrent().getValue();
        inputs.supplyCurrent = hoodMotor.getSupplyCurrent().getValue();
        inputs.speed = hoodMotor.getVelocity().getValue();
        inputs.isMotorConnected = hoodMotor.isConnected();
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
