package frc.robot.util.poseEstimation.extendedKalmanFilter;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface EKF<States, Inputs, Outputs> {

    void clearKalmanFilter();

    void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs);

    void setStateStdDevs(Matrix<N3, N1> stateStdDevs);

    void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters);

    void resetPose(Pose2d pose);

    void resetTranslation(Translation2d translation);

    void resetRotation(Rotation2d rotation);

    void getEstimatedPosition();

    Optional<Pose2d> sampleAt(double timestampSeconds);

    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

    void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);

    Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions);

    Pose2d updateWithTime(
            double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions);

    Matrix<N3, N1> getStateDeviations();

    Matrix<N3, N1> getVisionDeviations();

}
