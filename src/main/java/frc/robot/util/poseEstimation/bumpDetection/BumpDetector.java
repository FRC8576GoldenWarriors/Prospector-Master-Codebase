package frc.robot.util.poseEstimation.bumpDetection;

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

    private final Angle pitchThreshold = Degrees.of(11);//4
    private final Angle rollThreshold = Degrees.of(5);//4
    private final Angle pitchOffset = Degrees.of(0.527334);//-0.527334//1.845703+0.69);
    private final Angle rollOffset = Degrees.of(0);//-7.338867+3.258);
    private final Time debouncerTimeThreshold = Seconds.of(0.2);

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;

    private final Notifier signalUpdater;

    private final Debouncer rollSignalDebouncer;
    private final Debouncer pitchSignalDebouncer;

    private final Timer timeSinceBump;

    @AutoLogOutput
    private boolean isBumping = false;

    public BumpDetector(StatusSignal<Angle> pitchStatusSignal, StatusSignal<Angle> rollStatusSignal, Frequency period) {
        this.pitchSignal = pitchStatusSignal;
        this.rollSignal = rollStatusSignal;

        signalUpdater = new Notifier(this::updateInputs);

        rollSignalDebouncer = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);
        pitchSignalDebouncer = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);

        timeSinceBump = new Timer();

        signalUpdater.startPeriodic(period);
    }

    public void updateInputs() {
        isBumping = pitchSignalDebouncer.calculate(Math.abs(pitchSignal.getValue().minus(pitchOffset).in(Degrees)) > pitchThreshold.in(Degrees)) || rollSignalDebouncer.calculate(Math.abs(rollSignal.getValue().minus(rollOffset).in(Degrees)) > rollThreshold.in(Degrees));
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

        if (!timeSinceBump.isRunning() || timeSinceBumpSeconds > 7) {
            return Pair.of(0.0, 0.0);
        }

        double stdDev = Math.pow(Math.E, -(timeSinceBumpSeconds - 1));
        return Pair.of(stdDev, stdDev);
    }
}
