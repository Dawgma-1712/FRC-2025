package frc;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {
    public static class OperatorConstants {

        public static final Slot0Configs armConfigs = new Slot0Configs()
        .withKP(1).withKI(0).withKD(0);

        // Intake constants 
        public static final int intakeMotorID = 14;
        public static final double intakerMotorSpd = 0.6;
        public static final double intakeAngle = 45.0;
        public static final double stowAngle = 0.0;
        public static final double baseAngle = 0;

        // Crossbow constants
        public static final double topCrossbow = 3;
        public static final double bottomCrossbow = 0;
        public static final double crossbowMarginOfError = 1;
        public static final double crossbowSpeed = 0.5;     
        

        // Climber constants
        public static final double climberAngle = 90;
        public static final double increaseAmount = 1;
    }
}
