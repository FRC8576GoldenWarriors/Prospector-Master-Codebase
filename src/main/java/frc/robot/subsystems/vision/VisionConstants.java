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

import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-hub";
    public static String camera1Name = "limelight-left";
    public static String camera2Name = "limelight-tower";
    public static String camera3Name = "limelight-right";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Pose3d robotToCamera0 = new Pose3d(
        new Translation3d(
            0.027345,
            0.195258,
            0.664775
        ),
        new Rotation3d(
            Rotation2d.kZero
        )
    );
    public static Pose3d robotToCamera1 = new Pose3d(
        new Translation3d(
            -0.185425,
            0.301211,
            0.528772
        ),
        new Rotation3d(
            Rotation2d.kCCW_90deg
        )
    );
    public static Pose3d robotToCamera2 = new Pose3d(
        new Translation3d(
            -0.182284,
            -0.308071,
            0.381222
        ),
        new Rotation3d(
            Rotation2d.k180deg
        )
    );
    public static Pose3d robotToCamera3 = new Pose3d(
        new Translation3d(
            -0.297855,
            -0.287025,
            0.531059
        ),
        new Rotation3d(
            Rotation2d.kCW_90deg
        )
    );

    // Name to Transform Map
    public static HashMap<String, Pose3d> nameToPose3dHashMap = new HashMap<>(
        Map.ofEntries(
            Map.entry(camera0Name, robotToCamera0),
            Map.entry(camera1Name, robotToCamera1),
            Map.entry(camera2Name, robotToCamera2),
            Map.entry(camera3Name, robotToCamera3)
        )
    );


    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline= List.of(0.0931, 0.2991, 0.2431, 0.1059).stream().mapToDouble(Double::doubleValue).average().getAsDouble();//800 // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        0.0931, // Camera 0
        0.2991, // Camera 1
        0.2431, // Camera 2
        0.1059 // Camera 3
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.01;//0.2; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    // Time for internal IMU to converge after reset, in seconds
    public static final Time internalIMUConvergenceTime = Seconds.of(0.5);
}
