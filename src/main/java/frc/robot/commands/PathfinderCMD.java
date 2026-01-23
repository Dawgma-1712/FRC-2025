package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.Constants;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;


public class PathfinderCMD extends Command{

        private String targetPoseName;
        private Pose2d targetPose;
        private boolean isBlueSide = DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue;
        
        public static Pose2d CMAlgae;
        public static Pose2d CRAlgae;
        public static Pose2d CLAlgae;
        public static Pose2d FMAlgae;
        public static Pose2d FRAlgae;
        public static Pose2d FLAlgae;
        public static Pose2d processor;

        

    {
        if(isBlueSide){
            CMAlgae = Constants.OperatorConstants.blueCMAlgae; 
            CRAlgae = Constants.OperatorConstants.blueCRAlgae;
            CLAlgae = Constants.OperatorConstants.blueCLAlgae;
            FMAlgae = Constants.OperatorConstants.blueFMAlgae;
            FRAlgae = Constants.OperatorConstants.blueFRAlgae; 
            FLAlgae = Constants.OperatorConstants.blueFLAlgae; 
            processor = Constants.OperatorConstants.blueProcessor;
        }
        else{
            CMAlgae = Constants.OperatorConstants.redCMAlgae;
            CRAlgae = Constants.OperatorConstants.redCRAlgae;
            CLAlgae = Constants.OperatorConstants.redCLAlgae;
            FMAlgae = Constants.OperatorConstants.redFMAlgae;
            FRAlgae = Constants.OperatorConstants.redFRAlgae; 
            FLAlgae = Constants.OperatorConstants.redFLAlgae; 
            processor = Constants.OperatorConstants.redProcessor;
        }
    }

        public PathfinderCMD(String targetPoseName){
            this.targetPoseName = targetPoseName;

           
        }

        public void initialize(){
            switch (targetPoseName){
                case "CMAlgae":
                    targetPose=CMAlgae;
                    break;
                case "CRAlgae":
                    targetPose=CRAlgae;
                    break;
                case "CLAlgae":
                    targetPose=CLAlgae;
                    break;
                case "FMAlgae":
                    targetPose=FMAlgae;
                    break;
                case "FRAlgae":
                    targetPose=FRAlgae;
                    break;
                case "FLAlgae":
                    targetPose=FLAlgae;
                    break;
                case "Processor":
                    targetPose=processor;
                default:
                    System.out.println("Invalid Target Pose");
                    break;
            }
            
            //change to gameConstraints in a real game

            AutoBuilder.pathfindToPose(targetPose, Constants.OperatorConstants.gameConstraints, 0).schedule();
        }

        

        

        public void execute(){

        }

        public void end(){

        }

        public boolean isFinished(){
            return true;
        }

        public static Pose2d[] getAllianceReefPoses() {
            // Check alliance at method call time
            boolean isBlueSide = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        
            if (isBlueSide) {
                // Return array of Blue Poses (Assuming these are defined in Constants.java)
                return new Pose2d[]{
                    Constants.OperatorConstants.blueCMAlgae,
                    Constants.OperatorConstants.blueCRAlgae,
                    Constants.OperatorConstants.blueCLAlgae,
                    Constants.OperatorConstants.blueFMAlgae,
                    Constants.OperatorConstants.blueFRAlgae,
                    Constants.OperatorConstants.blueFLAlgae
                };
            } else {
                // Return array of Red Poses
                return new Pose2d[]{
                    Constants.OperatorConstants.redCMAlgae,
                    Constants.OperatorConstants.redCRAlgae,
                    Constants.OperatorConstants.redCLAlgae,
                    Constants.OperatorConstants.redFMAlgae,
                    Constants.OperatorConstants.redFRAlgae,
                    Constants.OperatorConstants.redFLAlgae
                };
            }
        }

}
