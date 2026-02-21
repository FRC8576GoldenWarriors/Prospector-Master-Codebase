package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final class HardwareConstants {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 8576;
  }

  public static final class PatternConfig {
    //Ready to shoot
    public static final LEDPattern GreenSolid = LEDPattern.solid(new Color(0, 255, 0));

    // Ddrivetrain not in range to shoot/climb align
    public static final LEDPattern WhiteBlink = LEDPattern.solid(Color.kWhite);
    public static final double WhiteBlinkSpeed = 0.075;

    // PurpleSoild, grond intake down
    public static final LEDPattern PurpleSolid = LEDPattern.solid(Color.kPurple);

    // Blue Slow blining  -climb activated
    public static final LEDPattern BlueBlink = LEDPattern.solid(Color.kBlue);
    public static final double BlueBlinkSPeed = 0.075;


    // red strobing - disabled
    public static final LEDPattern RebStrobing = LEDPattern.solid(Color.kRed);
    public static final double RebStrobingSpeed = 2.0;
    //ENABLED
    public static final LEDPattern YellowSolid = LEDPattern.solid(Color.kYellow);
  }
}
