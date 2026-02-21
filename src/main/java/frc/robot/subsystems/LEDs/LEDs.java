package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
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
  }

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

  public void blink(LEDPattern pattern, double speed) {
    // SmartDashboard.putString(getName(), "blink");
    // Logger.recordOutput(getName(), "blink");
    setPattern(pattern.blink(Seconds.of(speed)));
  }

  public void strobe(LEDPattern pattern, double speed) {
    // SmartDashboard.putString(getName(), "breathe");
    // Logger.recordOutput(getName(), "breathe");
    setPattern(pattern.breathe(Second.of(speed)).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  public void solid(LEDPattern pattern) {
    // SmartDashboard.putString(getName(), "Solid");
    // Logger.recordOutput(getName(), "Solid");
    setPattern(pattern);
  }

  @Override
  public void periodic() {

    if(DriverStation.isEnabled()){

      switch (state) {
        case Disabled:
          strobe(LEDConstants.PatternConfig.RebStrobing, LEDConstants.PatternConfig.RebStrobingSpeed);
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
        default:
          solid(LEDConstants.PatternConfig.YellowSolid);
          break;
      }



    }else{
      state = LEDStates.Disabled;
    }





    led.setData(buffer);

  }
}
