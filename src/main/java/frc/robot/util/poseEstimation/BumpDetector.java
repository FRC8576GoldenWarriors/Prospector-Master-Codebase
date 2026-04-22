package frc.robot.util.poseEstimation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;


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

    private final Angle pitchThreshold = Degrees.of(11);//4
    private final Angle rollThreshold = Degrees.of(5);//4
    private final Angle pitchOffset = DriveConstants.pitchGyroZero;//Degrees.of(0.527334);//-0.527334//1.845703+0.69);
    private final Angle rollOffset = DriveConstants.rollGyroZero;//Degrees.of(0);//-7.338867+3.258);
    private final Time debouncerTimeThreshold = Seconds.of(0.2);

    private final Supplier<Angle> pitchSupplier;
    private final Supplier<Angle> rollSupplier;

    private final Notifier signalUpdater;

    private final Debouncer rollSignalDebouncer;
    private final Debouncer pitchSignalDebouncer;

    private final Timer timeSinceBump;

    @AutoLogOutput
    private boolean isBumping = false;

    public BumpDetector(Supplier<Angle> pitchSupplier, Supplier<Angle> rollSupplier, Frequency period) {
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;

        signalUpdater = new Notifier(this::updateInputs);

        rollSignalDebouncer = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);
        pitchSignalDebouncer = new Debouncer(debouncerTimeThreshold.in(Seconds), DebounceType.kFalling);

        timeSinceBump = new Timer();

        signalUpdater.startPeriodic(period);
    }

    public void updateInputs() {
        isBumping = pitchSignalDebouncer.calculate(Math.abs(pitchSupplier.get().minus(pitchOffset).in(Degrees)) > pitchThreshold.in(Degrees)) || rollSignalDebouncer.calculate(Math.abs(rollSupplier.get().minus(rollOffset).in(Degrees)) > rollThreshold.in(Degrees));
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
