package frc.robot.util.poseEstimation.extendedKalmanFilter;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.BiFunction;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public class SwerveEKF<States extends Num, Inputs extends Num, Outputs extends Num> implements EKF<States, Inputs, Outputs> {

    // Underlying Extended Kalman Filter Class
    private ExtendedKalmanFilter<States, Inputs, Outputs> ekf;

    // Underlying Latency Compensator for the Kalman Filter
    private KalmanFilterLatencyCompensator<States, Inputs, Outputs> ekfLatencyCompensator;


    // Numerical amount of model States, Inputs, and Outputs
    private final Nat<States> numberOfStates;
    private final Nat<Inputs> numberOfInputs;
    private final Nat<Outputs> numberOfOutputs;

    // A vector-valued function of x and u that returns the derivative of the state vector.
    private final BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f;

    // A vector-valued function of x and u that returns the measurement vector.
    private final BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<Outputs, N1>> h;

    // The initial state and measurement standard deviations passed in
    private Matrix<States, N1> stateStandardDeviations;
    private Matrix<Outputs, N1> measurementStandardDeviations;


    // Constants
    private final Time dtSeconds = Milliseconds.of(20);
    private final Nat<N1> numberOfColumns = Nat.N1();

    public SwerveEKF(Nat<States> numberOfStates, Nat<Inputs> numberOfInputs, Nat<Outputs> numberOfOutputs, Matrix<States, N1> stateStandardDeviations, Matrix<Outputs, N1> measurementStandardDeviations) {

        this.numberOfStates = numberOfStates;
        this.numberOfInputs = numberOfInputs;
        this.numberOfOutputs = numberOfOutputs;

        this.f = (u, x) -> {
            double[] statesData = new double[numberOfStates.getNum()];

            // TODO Implement a method to return the derivative of the state vector

            return MatBuilder.fill(numberOfStates, numberOfColumns, statesData);
        };

        this.h = (u, x) -> {
            double[] outputData = new double[numberOfOutputs.getNum()];

            // TODO Implment a method to return the measurment vector

            return MatBuilder.fill(numberOfOutputs, numberOfColumns, outputData);
        };

        ekf =
            new ExtendedKalmanFilter<>(
                numberOfStates,
                numberOfInputs,
                numberOfOutputs,
                f,
                h,
                stateStandardDeviations,
                measurementStandardDeviations,
                dtSeconds.in(Seconds));

        ekfLatencyCompensator =
            new KalmanFilterLatencyCompensator<>();
    }

    @Override
    public void clearKalmanFilter() {
        ekf.reset();
        ekfLatencyCompensator.reset();
    }

    @Override
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVisionMeasurementStdDevs'");
    }

    @Override
    public void setStateStdDevs(Matrix<N3, N1> stateStdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStateStdDevs'");
    }

    @Override
    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void resetPose(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPose'");
    }

    @Override
    public void resetTranslation(Translation2d translation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetTranslation'");
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetRotation'");
    }

    @Override
    public void getEstimatedPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEstimatedPosition'");
    }

    @Override
    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'sampleAt'");
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addVisionMeasurement'");
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addVisionMeasurement'");
    }

    @Override
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    @Override
    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateWithTime'");
    }

    @Override
    public Matrix<N3, N1> getStateDeviations() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getStateDeviations'");
    }

    @Override
    public Matrix<N3, N1> getVisionDeviations() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVisionDeviations'");
    }

}
