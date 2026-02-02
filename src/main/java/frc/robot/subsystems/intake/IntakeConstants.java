package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class IntakeConstants {


    public class Hardware{
    public static int pivotID = 10;
        public static int rollerID = 11;
            public static int leftEncoderID = 1;
                        public static int rightEncoderID = 5;
        public static InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;
        public static InvertedValue rollerInvertedValue = InvertedValue.Clockwise_Positive;

    }

    public class Software{
        public static double leftZero = 0.0;
                public static double rightZero = 0.0;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kG = 0;
        public static double kV = 0;
        public static double kA = 0;

        public static double intakeUp = 0;
                public static double intakeDown = 0;


                                public static double rollerSpeed = 0;
        public static double intakeSoftStop = 0.0;


        public static Constraints profile = new Constraints(0.5, 0.75);

    }

}
