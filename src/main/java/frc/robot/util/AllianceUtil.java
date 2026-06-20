package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {

    private static final Supplier<Alliance> allianceSupplier = () -> {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    };

    private static final BooleanSupplier blueAllianceSupplier = () -> {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    };

    private static final BooleanSupplier redAllianceSupplier = () -> {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    };

    private static final BooleanSupplier hasAllianceSupplier = () -> {
        return DriverStation.getAlliance().isPresent();
    };

    @AutoLogOutput
    public static Alliance getCurrentAlliance() {
        return allianceSupplier.get();
    }

    public static boolean onBlueAlliance() {
        return blueAllianceSupplier.getAsBoolean();
    }

    public static boolean onRedAlliance() {
        return redAllianceSupplier.getAsBoolean();
    }

    public static Supplier<Alliance> getAllianceSupplier() {
        return allianceSupplier;
    }

    public static BooleanSupplier getBlueAllianceSupplier() {
        return blueAllianceSupplier;
    }

    public static BooleanSupplier getRedAllianceSupplier() {
        return redAllianceSupplier;
    }

    public static boolean hasAlliance() {
        return hasAllianceSupplier.getAsBoolean();
    }

    public static BooleanSupplier getHasAllianceSupplier() {
        return hasAllianceSupplier;
    }

}
