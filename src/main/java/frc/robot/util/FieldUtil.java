package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class FieldUtilConstants{
    public static Distance blueAllianceXThreshold = Inches.of(181.56);
    public static Distance redAlliannceXThreshold = Inches.of(650.12-181.56);//Minimum to be in red zone
}
public class FieldUtil{
    public static enum fieldPosition{
        RedZone,
        BlueZone,
        NeutralZone
    }
    private static Supplier<Pose2d> poseSupplier;
    public FieldUtil(Supplier<Pose2d> poseSupplier){
        FieldUtil.poseSupplier = poseSupplier;
    }
    public static fieldPosition getFieldPosition(){
        if(poseSupplier.get().getX()<FieldUtilConstants.blueAllianceXThreshold.in(Meters)){
            return fieldPosition.BlueZone;
        }else if(poseSupplier.get().getX()>FieldUtilConstants.redAlliannceXThreshold.in(Meters)){
            return fieldPosition.RedZone;
        }else{
            return fieldPosition.NeutralZone;
        }
    }
    @AutoLogOutput (key = "FieldUtil/On Alliance Side")
    public static boolean isOnAllianceSide(){
        fieldPosition position = getFieldPosition();
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (position==fieldPosition.BlueZone&&currentAlliance==Alliance.Blue)||(position==fieldPosition.RedZone&&currentAlliance==Alliance.Red);
    }

}
