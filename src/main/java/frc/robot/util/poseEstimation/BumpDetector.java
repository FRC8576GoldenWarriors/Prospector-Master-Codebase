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

public class BumpDetector {

    private final Angle pitchThreshold = Degrees.of(4);
    private final Angle rollThreshold = Degrees.of(4);
    private final Angle pitchOffset = Degrees.of(3.127);
    private final Time debouncerTimeThreshold = Seconds.of(0.4);

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;

    private final Notifier signalUpdater;

    private final Debouncer yawSignalDebouncer;
    private final Debouncer pitchSignalDebouncer;

    private final Timer timeSinceBump;

    @AutoLogOutput
    private boolean isBumping = false;

    public BumpDetector(StatusSignal<Angle> pitchStatusSignal, StatusSignal<Angle> rollStatusSignal, Frequency period) {
        this.pitchSignal = pitchStatusSignal;
        this.rollSignal = rollStatusSignal;

        signalUpdater = new Notifier(this::updateInputs);

        yawSignalDebouncer = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);
        pitchSignalDebouncer = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);

        timeSinceBump = new Timer();

        signalUpdater.startPeriodic(period);
    }

    public void updateInputs() {
        isBumping = pitchSignalDebouncer.calculate(Math.abs(pitchSignal.getValue().minus(pitchOffset).in(Degrees)) > pitchThreshold.in(Degrees)) || yawSignalDebouncer.calculate(Math.abs(rollSignal.getValue().in(Degrees)) > rollThreshold.in(Degrees));
        if(isBumping) {
            if(!timeSinceBump.isRunning())
                timeSinceBump.start();
            else
                timeSinceBump.reset();

        }
    }

    public boolean isBumping() {
        return isBumping;
    }

    public Pair<Double, Double> getBumpSTDDevs(){
        double timeSinceBumpSeconds = timeSinceBump.get();

        double xStdDev = (!timeSinceBump.isRunning()) ? 0.0 : Math.pow(Math.E, -(timeSinceBumpSeconds - 1));
        double yStdDev = (!timeSinceBump.isRunning()) ? 0.0 : Math.pow(Math.E, -(timeSinceBumpSeconds - 1));

        return Pair.of(xStdDev, yStdDev);
    }
}
