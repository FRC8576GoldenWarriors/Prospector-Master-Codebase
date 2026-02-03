// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final TimeInterpolatableBuffer<Translation3d> collisonVector = TimeInterpolatableBuffer.createBuffer(2);

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(odometryFrequency);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    public void resetHeading(double headingDegrees) {
        pigeon.setYaw(headingDegrees);
    }

    public Translation3d getForceVector() {
        return new Translation3d(VecBuilder.fill(
            pigeon.getAccelerationX().getValueAsDouble(),
            pigeon.getAccelerationY().getValueAsDouble(),
            pigeon.getAccelerationZ().getValueAsDouble()));
    }

    public Translation3d getForceVectorDerivative() {
        var firstSample = collisonVector.getSample(Timer.getFPGATimestamp() - 0.1);
        var secondSample = collisonVector.getSample(Timer.getFPGATimestamp() - 0.05);

        if(firstSample.isPresent() & secondSample.isPresent()) {
            return new Translation3d();
        }

        return secondSample.get().minus(firstSample.get());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.xVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityXWorld().getValueAsDouble());
        inputs.yVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityYWorld().getValueAsDouble());
        inputs.zVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        collisonVector.addSample(Timer.getFPGATimestamp(), this.getForceVector());
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
