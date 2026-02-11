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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

import java.util.Queue;


/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    public final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<Angle> roll = pigeon.getRoll();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final StatusSignal<LinearAcceleration> accelerationX = pigeon.getAccelerationX();
    private final StatusSignal<LinearAcceleration> accelerationY = pigeon.getAccelerationY();
    private final StatusSignal<LinearAcceleration> accelerationZ = pigeon.getAccelerationZ();

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

    public StatusSignal<LinearAcceleration> getXAccelerationStatusSignal() {
        return accelerationX;
    }

    public StatusSignal<LinearAcceleration> getYAccelerationStatusSignal() {
        return accelerationY;
    }

    public StatusSignal<Angle> getPitchStatusSignal() {
        return pitch;
    }

    public StatusSignal<Angle> getRollStatusSignal() {
        return roll;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity, accelerationX, accelerationY, accelerationZ, pitch, roll).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.pitchDegrees = Math.abs(pitch.getValue().minus(Degrees.of(3.127)).in(Degrees));
        inputs.rollDegrees = Math.abs(roll.getValue().plus(Degrees.of(0.747)).in(Degrees));
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.xVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityXWorld().getValueAsDouble());
        inputs.yVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityYWorld().getValueAsDouble());
        inputs.zVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
        inputs.xAccelerationMetersPerSecondPerSecond = pigeon.getAccelerationX().getValue().in(MetersPerSecondPerSecond);
        inputs.yAccelerationMetersPerSecondPerSecond = pigeon.getAccelerationY().getValue().in(MetersPerSecondPerSecond);
        inputs.zAccelerationMetersPerSecondPerSecond = pigeon.getAccelerationZ().getValue().minus(MetersPerSecondPerSecond.of(9.81)).in(MetersPerSecondPerSecond);
        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
