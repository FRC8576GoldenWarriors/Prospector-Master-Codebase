package frc.robot.util.poseEstimation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;

public class BumpDetector {

    private final Angle pitchThreshold = Degrees.of(4);
    private final Angle rollThreshold = Degrees.of(4);
    private final Angle pitchOffset = Degrees.of(3.127);
    private final Time debouncerTimeThreshold = Seconds.of(0.4);

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;

    private final Notifier signalUpdater = new Notifier(() -> updateInputs());
    private final Debouncer signal = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);

    @AutoLogOutput
    public boolean isBumping = false;

    @AutoLogOutput
    public Time timeOfBump = Seconds.of(-1);

    public BumpDetector(Pair<StatusSignal<Angle>, StatusSignal<Angle>> pitchAndRollSignalPair, Frequency period) {
        this.pitchSignal = pitchAndRollSignalPair.getFirst();
        this.rollSignal = pitchAndRollSignalPair.getSecond();
        signalUpdater.startPeriodic(period);
    }

    public void updateInputs() {
        isBumping = signal.calculate(Math.abs(pitchSignal.getValue().minus(pitchOffset).in(Degrees)) > pitchThreshold.in(Degrees) || Math.abs(rollSignal.getValue().in(Degrees)) > rollThreshold.in(Degrees));
        if(isBumping) {
            timeOfBump = Seconds.of(Timer.getFPGATimestamp());
        }
    }

    public boolean isBumping() {
        return isBumping;
    }

    public Time getTimeSincBump(){
        return Seconds.of(Timer.getFPGATimestamp()).minus(timeOfBump);
    }

    public Pair<Double, Double> getBaseSTDDevs(){
        double timeSinceBump = this.getTimeSincBump().in(Seconds);

        double xStdDev = (timeSinceBump < 0) ? DriveConstants.baseXDriveSTDEV : Math.pow(Math.E, -(timeSinceBump-1) + DriveConstants.baseXDriveSTDEV);
        double yStdDev = (timeSinceBump < 0) ? DriveConstants.baseYDriveSTDEV : Math.pow(Math.E, -(timeSinceBump-1) + DriveConstants.baseYDriveSTDEV);

        return Pair.of(xStdDev, yStdDev);
    }
}
