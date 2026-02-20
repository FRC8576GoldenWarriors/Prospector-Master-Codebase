package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public interface ClimberIO {
    default void updateInputs(ClimberIOInputs inputs) {}

  @AutoLog
 public class ClimberIOInputs {
    public Voltage pivotVoltage = Volts.of(0.0);
    public Current pivotCurrent = Amps.of(0);
     public Current pivotSupplyCurrent = Amps.of(0);
    public AngularVelocity pivotRPS = RotationsPerSecond.of(0);
    public boolean pivotConnected = false;



    public boolean encoderConnected = false;


    public double encoderRotations = 0.0;

    public boolean backRightCANRangeConnected =   false;
    public boolean backLeftCANRangeConnected =   false;
    public boolean frontRightANRangeConnected =   false;
    public boolean frontLeftCANRangeConnected =   false;
    public boolean rightFacingCANRangeConnected = false;


    public double backRightCANRangeDistance =   0 ;
    public double backLeftCANRangeDistance =    0;
    public double frontRightANRangeDistance =   0 ;
    public double frontLeftCANRangeDistance =   0 ;
    public double rightFacingCANRangeDistance =   0 ;

    public boolean photoelectricDetected = false;


}

    default void setPivotVoltage(double voltage) {}

    default void setPivotSpeed(double speed) {}

    default boolean detected(){
        return false;
    }

}
