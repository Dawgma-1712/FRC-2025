package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.Constants;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;


public class PathfinderCMD extends Command{
        private Pose2d targetPose;
        private boolean isBlueSide = DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue;
        
        public PathfinderCMD(Pose2d targetPose){
            this.targetPose=targetPose;
        }

        


        



    
}
