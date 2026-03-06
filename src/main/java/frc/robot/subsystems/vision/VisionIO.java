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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public String name = "";
        public int imuDataLength = 0;

        public Rotation2d imuRobotYaw = new Rotation2d();
        public Angle imuRoll = Degrees.of(0);
        public Angle imuPitch = Degrees.of(0);
        public Angle imuInternalYaw = Degrees.of(0);
        public AngularVelocity imuRollVelocity = DegreesPerSecond.of(0);
        public AngularVelocity imuPitchVelocity = DegreesPerSecond.of(0);
        public AngularVelocity imuYawVelocity = DegreesPerSecond.of(0);
        public AngularAcceleration imuRollAcceleration = DegreesPerSecondPerSecond.of(0);
        public AngularAcceleration imuPitchAcceleration = DegreesPerSecondPerSecond.of(0);
        public AngularAcceleration imuYawAcceleration = DegreesPerSecondPerSecond.of(0);
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation. */
    record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            //double[] standardDeviations,
            double angularVelocity,
            PoseObservationType type) {}

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    enum IMUMode {
        EXTERNAL_ONLY(0),
        EXTERNAL_SEED(1),
        INTERNAL_ONLY(2),
        INTERNAL_MT1_ASSIST(3),
        INTERNAL_EXTERNAL_ASSIST(4);

        private final int mode;
        IMUMode(int mode) {
            this.mode = mode;
        }

        public int getMode() {
            return mode;
        }
    }

    default void resetHeading() {}

    default void updateInputs(VisionIOInputs inputs) {}
}
