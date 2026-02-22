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


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-barge";
    public static String camera1Name = "limelight-reef";
    public static String camera2Name = "limelight-backup";
    public static String camera3Name = "limelight-side";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // public static Pose3d robotToCamera0 = PoseUtil.onshapeToLimelightPose(Inches.of(6.018), Inches.of(0.208), Inches.of(-30.998), Degrees.of(-1.2), Degrees.of(28.7), Degrees.of(0));
    // public static Pose3d robotToCamera1 = PoseUtil.onshapeToLimelightPose(Inches.of(3.025), Inches.of(-8.490), Inches.of(-9.252), Degrees.of(0), Degrees.of(28.2), Degrees.of(180));

    // public static Pose3d robotToCamera2 = PoseUtil.onshapeToLimelightPose(Inches.of(0.253), Inches.of(8.023), Inches.of(-24.234), Degrees.of(-1.6), Degrees.of(5.3), Degrees.of(180));
    // public static Pose3d robotToCamera3 = PoseUtil.onshapeToLimelightPose(Inches.of(-16.206), Inches.of(-5.881), Inches.of(-22.285), Degrees.of(-6.6), Degrees.of(11), Degrees.of(90));

    // Name to Transform Map
    // public static HashMap<String, Pose3d> nameToPose3dHashMap = new HashMap<>(
    //     Map.ofEntries(
    //         Map.entry(camera0Name, robotToCamera0),
    //         Map.entry(camera1Name, robotToCamera1),
    //         Map.entry(camera2Name, robotToCamera2),
    //         Map.entry(camera3Name, robotToCamera3)
    //     )
    // );


    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 1;//800 // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.1;//0.2; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    // Time for internal IMU to converge after reset, in seconds
    public static final Time internalIMUConvergenceTime = Seconds.of(0.5);
}
