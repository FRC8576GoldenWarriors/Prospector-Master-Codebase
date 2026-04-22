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
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

import java.util.Queue;


/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    public final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<AngularVelocity> rollVelocity;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final StatusSignal<LinearAcceleration> accelerationZ;
    //private final Stack<Rotation2d> gyroStack = new Stack<Rotation2d>();

    public GyroIOPigeon2() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.GyroTrim.withGyroScalarZ(DriveConstants.gyroTrimDegreesPerRotation);
        pigeon.getConfigurator().apply(config);
        pigeon.getConfigurator().setYaw(0.0);

        yaw = pigeon.getYaw();
        roll = pigeon.getRoll();
        pitch = pigeon.getPitch();

        yawVelocity = pigeon.getAngularVelocityZWorld();
        rollVelocity = pigeon.getAngularVelocityYWorld();
        pitchVelocity = pigeon.getAngularVelocityXWorld();

        accelerationX = pigeon.getAccelerationX();
        accelerationY = pigeon.getAccelerationY();
        accelerationZ = pigeon.getAccelerationZ();

        yaw.setUpdateFrequency(odometryFrequency);
        pigeon.optimizeBusUtilization(Hertz.of(0));
        BaseStatusSignal.setUpdateFrequencyForAll(
            updateFrequency,
            roll,
            pitch,
            yawVelocity,
            rollVelocity,
            pitchVelocity,
            accelerationX,
            accelerationY,
            accelerationZ);
            StatusSignal<Double> random = new StatusSignal<Double>(new DeviceIdentifier(0,"SimGyro",CANBus.roboRIO()), 1, "Random", null, Double.class, (doub)->{return Double.valueOf(doub);});
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    public void resetHeading(double headingDegrees) {
        pigeon.setYaw(headingDegrees);
    }

    public LinearAcceleration getXAccelerationStatusSignal() {
        return accelerationX.getValue();
    }

    public LinearAcceleration getYAccelerationStatusSignal() {
        return accelerationY.getValue();
    }

    public Angle getPitchStatusSignal() {
        return pitch.getValue();
    }

    public Angle getRollStatusSignal() {
        return roll.getValue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            yaw,
            pitch,
            roll,
            yawVelocity,
            pitchVelocity,
            rollVelocity,
            accelerationX,
            accelerationY,
            accelerationZ);

        inputs.connected = pigeon.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.pitchDegrees = Math.abs(pitch.getValue().minus(Degrees.of(3.127)).in(Degrees));
        inputs.rollDegrees = Math.abs(roll.getValue().plus(Degrees.of(0.747)).in(Degrees));
        inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond);
        inputs.xVelocityRadPerSec = pitchVelocity.getValue().in(RadiansPerSecond);
        inputs.yVelocityRadPerSec = rollVelocity.getValue().in(RadiansPerSecond);
        inputs.zVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond);
        inputs.xAccelerationMetersPerSecondPerSecond = accelerationX.getValue().in(MetersPerSecondPerSecond);
        inputs.yAccelerationMetersPerSecondPerSecond = accelerationY.getValue().in(MetersPerSecondPerSecond);
        inputs.zAccelerationMetersPerSecondPerSecond = accelerationZ.getValue().minus(MetersPerSecondPerSecond.of(9.81)).in(MetersPerSecondPerSecond);

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
    // public Rotation2d getGyroYaw(Rotation2d yawPosition){

    // }
}
