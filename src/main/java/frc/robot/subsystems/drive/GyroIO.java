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

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;


import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double pitchDegrees = 0;
        public double rollDegrees = 0;
        public double yawVelocityRadPerSec = 1;
        public double xVelocityRadPerSec = 1;
        public double yVelocityRadPerSec = 1;
        public double zVelocityRadPerSec = 1;
        public double xAccelerationMetersPerSecondPerSecond = 1;
        public double yAccelerationMetersPerSecondPerSecond = 1;
        public double zAccelerationMetersPerSecondPerSecond = 1;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default StatusSignal<LinearAcceleration> getXAccelerationStatusSignal() {
        return null;
    }

    public default StatusSignal<LinearAcceleration> getYAccelerationStatusSignal() {
        return null;
    }

    public default StatusSignal<Angle> getPitchStatusSignal() {
        return null;
    }

    public default StatusSignal<Angle> getRollStatusSignal() {
        return null;
    }

    public default void resetHeading(double headingDegrees) {}
}
