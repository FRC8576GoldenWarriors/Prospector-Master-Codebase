package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
    @Override
    public StatusSignal<Angle> getPitchStatusSignal() {
        // TODO Auto-generated method stub
        return GyroIO.super.getPitchStatusSignal();
    }
    @Override
    public StatusSignal<Angle> getRollStatusSignal() {
        // TODO Auto-generated method stub
        return GyroIO.super.getRollStatusSignal();
    }
    @Override
    public StatusSignal<LinearAcceleration> getXAccelerationStatusSignal() {
        // TODO Auto-generated method stub
        return GyroIO.super.getXAccelerationStatusSignal();
    }
    @Override
    public StatusSignal<LinearAcceleration> getYAccelerationStatusSignal() {
        // TODO Auto-generated method stub
        return GyroIO.super.getYAccelerationStatusSignal();
    }
}
