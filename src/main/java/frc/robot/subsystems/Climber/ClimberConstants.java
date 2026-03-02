package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ClimberConstants {


    public class Hardware{
        public static int pivotID = 50;
        public static int encoderID = 4;
        public static InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;

        public static int backRightCANRange =   1 ;
        public static int backLeftCANRange =    2;
        public static int frontRightANRange =   3 ;
        public static int frontLeftCANRange =   4 ;
        public static int rightFacingCANRange =   5 ;
                public static int photoelectricID =   5;


    }

    public class Software{
        public static double encoderZero;
        public static boolean encoderInvert = true;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kG = 0.0;
        public static double kV = 0;
        public static double kA = 0;

        public static double autoClimb = 0.0;
        public static double teleClimb =0.0;


        public static Constraints profile = new Constraints(3.78, 3);

    }

}
