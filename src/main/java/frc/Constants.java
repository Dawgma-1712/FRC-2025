package frc;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {
    public static class OperatorConstants {

        public static final Slot0Configs armConfigs = new Slot0Configs()
        .withKP(1).withKI(0).withKD(0);

        // Motor IDs
        public static final int intakeMotorID = 14;
        public static final int crossbowMotorID = 15;
        public static final int climbingMotorID1 = 16;
        public static final int climbingMotorID2 = 17;
        public static final int intakeAngleID = 25; // used to be 13 but that's cursed apparently??

        // Intake constants 
        public static final double intakerMotorSpd = 0.6;
        public static final double intakeAngle = 55.0;
        public static final double stowAngle = 0.0;
        public static final double baseAngle = 0;
        public static final double topSwitchPosition = -20;
        public static final double bottomSwitchPosition = 59.5;

        // Crossbow constants
        public static final double topCrossbow = 20000;
        public static final double bottomCrossbow = 0;
        public static final double crossbowMarginOfError = 1;
        public static final double crossbowSpeed = 0.5;     
        

        // Climber constants
        public static final double climberAngle = 90;
        public static final double increaseAmount = 1;
    }
}
