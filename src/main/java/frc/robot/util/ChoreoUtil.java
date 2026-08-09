package frc.robot.util;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class ChoreoUtil {

    private static final BooleanSupplier onRedAlliance = AllianceUtil.getRedAllianceSupplier();

    public static AutoTrajectory flipAcrossDiagonal(AutoTrajectory trajectory) {
        return (onRedAlliance.getAsBoolean()) ? trajectory.mirrorX().mirrorY() : trajectory;
    }

    public static AutoTrajectory flipAcrossAlliance(AutoTrajectory trajectory) {
        return (onRedAlliance.getAsBoolean()) ? trajectory.mirrorX() : trajectory;
    }

    public static AutoTrajectory flipAcrossMidline(AutoTrajectory trajectory, boolean shouldFlip) {
        return (shouldFlip) ? trajectory.mirrorY() : trajectory;
    }

    public static AutoTrajectory loadAndFlipDiagonal(AutoRoutine routine, String trajectoryName) {
        return flipAcrossDiagonal(routine.trajectory(trajectoryName));
    }

    public static AutoTrajectory loadAndFlipMidline(AutoRoutine routine, String trajectoryName, boolean shouldFlip) {
        return flipAcrossMidline(routine.trajectory(trajectoryName), shouldFlip);
    }

    public static AutoTrajectory loadAndFlipMidlineAndDiagonal(AutoRoutine routine, String trajectoryName, boolean shouldFlipOverMidline) {
        return flipAcrossDiagonal(loadAndFlipMidline(routine, trajectoryName, shouldFlipOverMidline));
    }

    public static AutoTrajectory loadAndFlipMidlineAndDiagonal(AutoRoutine routine, String routineName, String trajectoryName) {
        return flipAcrossDiagonal(loadAndFlipMidline(routine, trajectoryName, routineName.toLowerCase().contains("left")));
    }



}
