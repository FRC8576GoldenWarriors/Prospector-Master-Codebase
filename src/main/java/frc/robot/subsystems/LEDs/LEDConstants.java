package frc.robot.subsystems.LEDs;

//import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  // public static final LoggedNetworkNumber redValue = new LoggedNetworkNumber("Red Value", 0);
  // public static final LoggedNetworkNumber greenValue = new LoggedNetworkNumber("Green Value", 0);
  // public static final LoggedNetworkNumber blueValue = new LoggedNetworkNumber("Blue Value", 0);

  public static final class HardwareConstants {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 8;
  }

  public static final class PatternConfig {
    //Ready to shoot
    public static final Color GreenSolid = ((Color.kGreen));//LEDPattern.solid(new Color(0, 255, 0));

    // Ddrivetrain not in range to shoot/climb align
    public static final Color WhiteBlink =(Color.kWhite);
    public static final double WhiteBlinkSpeed = 0.075;

    // PurpleSoild, grond intake down
    public static final Color PurpleSolid = ((Color.kPurple));

    // Blue Slow blining  -climb activated
    public static final Color BlueBlink = (Color.kBlue);
    public static final double BlueBlinkSPeed = 0.075;


    // red strobing - disabled
    public static final Color RebStrobing = ((Color.kRed));
    public static final double RebStrobingSpeed = 2.0;
    //ENABLED
    public static final Color YellowSolid = ((new Color(255,150,11)));

  }
}
