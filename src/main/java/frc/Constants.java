package frc;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
        public static final double intakeAngle = 58.0;
        public static final double stowAngle = 0.0;
        public static final double baseAngle = 0;
        public static final double topSwitchPosition = -20;
        public static final double bottomSwitchPosition = 59.5;
        public static final double maxPosition = 65;
        public static final double dereefAngle = 25;

        // Crossbow constants
        public static final double topCrossbow = 25;
        public static final double bottomCrossbow = 0;
        public static final double crossbowMarginOfError = 1;
        public static final double crossbowSpeed = 0.4;
        // Sensor IDs
        public static final int crossbowMagLimitSwitchID = 3;
        public static final int crossbowLimitSwitchID = 0;     
        

        // Climber constants
        public static final double climberAngle = 90;
        public static final double increaseAmount = 1;

        public static int blueOffset = 135;

        //Target Poses constants


        // public static final Pose2d blueCMAlgae = new Pose2d(3.231,4.010,Rotation2d.fromDegrees(-56.135));
        // public static final Pose2d blueCRAlgae = new Pose2d(3.862,2.943,Rotation2d.fromDegrees(125.341));
        // public static final Pose2d blueCLAlgae = new Pose2d(3.847,5.077,Rotation2d.fromDegrees(-72.255));
        // public static final Pose2d blueFMAlgae = new Pose2d(5.725,4.025,Rotation2d.fromDegrees(-109.185));
        // public static final Pose2d blueFRAlgae = new Pose2d(5.094,2.943,Rotation2d.fromDegrees(158.949));
        // public static final Pose2d blueFLAlgae = new Pose2d(5.109,5.107,Rotation2d.fromDegrees(136.755));

        public static final Pose2d blueCMAlgae = new Pose2d(3.231,4.010,Rotation2d.fromDegrees(180));
        public static final Pose2d blueCRAlgae = new Pose2d(3.862,2.943,Rotation2d.fromDegrees(60));
        public static final Pose2d blueCLAlgae = new Pose2d(3.847,5.077,Rotation2d.fromDegrees(-60));
        public static final Pose2d blueFMAlgae = new Pose2d(5.725,4.025,Rotation2d.fromDegrees(180));
        public static final Pose2d blueFRAlgae = new Pose2d(5.094,2.943,Rotation2d.fromDegrees(-60));
        public static final Pose2d blueFLAlgae = new Pose2d(5.109,5.107,Rotation2d.fromDegrees(60));

        public static final Pose2d redCMAlgae = new Pose2d(17.53-3.231,8.03-4.010,Rotation2d.fromDegrees(0));
        public static final Pose2d redCRAlgae = new Pose2d(17.53-3.862,8.03-2.943,Rotation2d.fromDegrees(180+60));
        public static final Pose2d redCLAlgae = new Pose2d(17.53-3.847,8.03-5.077,Rotation2d.fromDegrees(180-60));
        public static final Pose2d redFMAlgae = new Pose2d(17.53-5.725,8.03-4.025,Rotation2d.fromDegrees(0));
        public static final Pose2d redFRAlgae = new Pose2d(17.53-5.094,8.03-2.943,Rotation2d.fromDegrees(180-60));
        public static final Pose2d redFLAlgae = new Pose2d(17.53-5.109,8.03-5.107,Rotation2d.fromDegrees(180+60));

        public static final Pose2d blueProcessor = new Pose2d(5.975,0.570,Rotation2d.fromDegrees(-90));
        public static final Pose2d redProcessor = new Pose2d(17.53-5.975,8.03-0.570,Rotation2d.fromDegrees(180+90));


        //Constraints constant:
        public static final PathConstraints testingConstraints = new PathConstraints(.3, .4, Units.degreesToRadians(54), Units.degreesToRadians(72));
        public static final PathConstraints gameConstraints = new PathConstraints(3, 4, Units.degreesToRadians(540), Units.degreesToRadians(720));



        // Auto constants
        public static final int drive_kP = 10;
        public static final int drive_kI = 0;
        public static final int drive_kD = 0;

        public static final int rotation_kP = 7;
        public static final int rotation_kI = 0;
        public static final int rotation_kD = 0;

        public static final Double[] id_Blue = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0};
    public static final Double[] id_Red = {13.0, 12.0, 16.0, 15.0, 14.0, 19.0, 18.0, 17.0, 22.0, 21.0, 20.0};

    public static final double yDisplace = 0;
    public static final double llAngle = 0;

    public static final int idsLength = 22;

    public static final double[] aprilTagX = {657.37, 657.37, 455.15, 365.20, 365.20, 530.49, 546.87, 530.49, 497.77, 481.39, 497.77, 33.51, 33.51, 325.68, 325.68, 235.73, 160.39, 144.00, 160.39, 193.10, 209.49, 193.10};
    public static final double[] goalX = {-12, -12, -12, -12, -12, -12, 12, 12, -12, -12, -12, -12, -12, -12, -12, -12};
    public static final double[] aprilTagY = {25.8, 291.2, 317.15, 241.64, 75.39, 130.17, 158.5, 186.83, 186.83, 158.5, 130.17, 25,8, 291.2, 241.64, 75.39, -0.15, 130.17, 158.5, 186.83, 186.83, 158.5, 130.17};
    public static final double[] aprilTagZ = {58.5, 58.5, 51.25, 73.54, 73.54, 12.13, 12.13, 12.13, 12.13, 12.13, 12.13, 58.5, 58.5, 73.54, 73.54, 51.25, 12.13, 12.13, 12.13, 12.13, 12.13, 12.13};
    public static final double[] aprilTagYaw = {126, 234, 270, 0, 0, 300, 0, 60, 120, 180, 240, 54, 306, 180, 180, 90, 240, 180, 120, 60, 0, 300};

    public static final double X_REEF_ALIGNMENT_P = 0.0; //2.0
	public static final double Y_REEF_ALIGNMENT_P = 1.0; //2.0
	public static final double ROT_REEF_ALIGNMENT_P = 0.0;//0.075

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;
	public static final double X_SETPOINT_REEF_ALIGNMENT = -1.5;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

	public static final double DONT_SEE_TAG_WAIT_TIME = 0.5;
	public static final double POSE_VALIDATION_TIME = 0.3;
    }
}
