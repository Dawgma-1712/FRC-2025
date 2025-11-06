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
        
        private Pose2d CMAlgae;
        private Pose2d CRAlgae;
        private Pose2d CLAlgae;
        private Pose2d FMAlgae;
        private Pose2d FRAlgae;
        private Pose2d FLAlgae;
        private Pose2d processor;

        
        public PathfinderCMD(String targetPoseName){
            targetPoseName=this.targetPoseName;

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
                case "processor":
                    targetPose=processor;
                default:
                    System.out.println("Invalid Target Pose");
                    break;
            }
            
            //change to gameConstraints in a real game
            AutoBuilder.pathfindToPose(targetPose, Constants.OperatorConstants.testingConstraints, 0);
        }

        

        public void execute(){

        }

        public void end(){

        }

        public boolean isFinished(){
            return true;
        }


    
}
