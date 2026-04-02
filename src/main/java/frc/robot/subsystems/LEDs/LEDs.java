package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.*;


import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class LEDs extends SubsystemBase {

  private int port;
  private int length;
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public enum LEDStates{
    OutOfRange,
    ReadyToShoot,
    GroundIntake,
    ClimbStart,
    ClimbEnd,
    Disabled,
    Enabled,
    Idle,
  }

  @AutoLogOutput (key = "LEDs/Wanted State")
  public LEDStates state = LEDStates.Disabled;

  public LEDs() {
    this.port = LEDConstants.HardwareConstants.LED_PORT;
    this.length = LEDConstants.HardwareConstants.LED_LENGTH;

    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(length);
    led.start();
  }

  public int getPort() {
    return this.port;
  }

  public int getLength() {
    return this.length;
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  // public void rainbowScroll() {

  //   Map<Double, Color> maskSteps =
  //       Map.of(0.0, Color.kWhite, LEDConstants.PatternConfig.LED_RAINBOW_SCROLL_SIZE, Color.kBlack);
  //   LEDPattern base = LEDPattern.rainbow(255, 255);
  //   LEDPattern mask =
  //       LEDPattern.steps(maskSteps)
  //           .scrollAtRelativeSpeed(
  //               Percent.per(Second).of(LEDConstants.PatternConfig.LED_RAINBOW_SCROLL_SPEED));

  //   LEDPattern pattern = base.mask(mask);
  //   // pattern = base.mask(progress);

  //   setPattern(pattern);
  // }

  public void setState(LEDStates state){
    this.state = state;
  }

  public static LEDPattern redGreenSwap(Color color) {
    return LEDPattern.solid((new Color(color.green,color.red,color.blue)));
  }

  public void blink(Color color, double speed) {
    LEDPattern pattern = redGreenSwap(color);
    // SmartDashboard.putString(getName(), "blink");
    // Logger.recordOutput(getName(), "blink");

    setPattern(pattern.blink(Seconds.of(speed)));
  }

  public void strobe(Color color, double speed) {
    LEDPattern pattern = redGreenSwap(color);
    // SmartDashboard.putString(getName(), "breathe");
    // Logger.recordOutput(getName(), "breathe");
    setPattern(pattern.breathe(Second.of(speed)).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  public void solid(Color color) {
    // SmartDashboard.putString(getName(), "Solid");
    // Logger.recordOutput(getName(), "Solid");
    LEDPattern pattern = redGreenSwap(color);
    setPattern(pattern);
  }

  @Override
  public void periodic() {


      switch (state) {
        case Disabled:
          //strobe(LEDConstants.PatternConfig.RebStrobing, LEDConstants.PatternConfig.RebStrobingSpeed);
          solid(LEDConstants.PatternConfig.RebStrobing);
          break;
        case OutOfRange:
          blink(LEDConstants.PatternConfig.WhiteBlink, LEDConstants.PatternConfig.WhiteBlinkSpeed);
          break;
        case ReadyToShoot:
          solid(LEDConstants.PatternConfig.GreenSolid);
          break;
        case GroundIntake:
          solid(LEDConstants.PatternConfig.PurpleSolid);
          break;
        case ClimbStart:
          blink(LEDConstants.PatternConfig.BlueBlink, LEDConstants.PatternConfig.BlueBlinkSPeed);
          break;
        case ClimbEnd:
          solid(LEDConstants.PatternConfig.BlueBlink);
          break;
        case Idle:
          solid(LEDConstants.PatternConfig.YellowSolid);
          break;
        default:
          solid(LEDConstants.PatternConfig.YellowSolid);
          break;



    }
    led.setData(buffer);

  }
}
