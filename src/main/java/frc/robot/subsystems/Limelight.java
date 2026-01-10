// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;
import frc.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;


public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = true;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  
  // StructPublisher<Pose3d> publisher3D = NetworkTableInstance.getDefault().getStructTopic("AprilTag", Pose3d.struct).publish();

  StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTagArray", Pose3d.struct).publish();


  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

  @Override
  public void periodic() {
    if (enable) {
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
      Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.LimelightResults result =
          LimelightHelpers.getLatestResults(ll);
      if (result.valid) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
        if (field.isPoseWithinArea(botpose)) {
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || trust
              || result.targets_Fiducials.length > 1) {
            drivetrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(confidence, confidence, .01));
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      }

      
      SmartDashboard.putNumber("Distance to Algae", getAlgaeDistance());
      Pose2d algaePose = drivetrain.getState().Pose;
      algaePose.transformBy(new Transform2d(getAlgaeDistance(), 0, new Rotation2d()));
      drivetrain.setAlgaePose(algaePose);

      arrayPublisher.set(aprilTagsVisible());
    }
  }


  public Pose3d[] aprilTagsVisible(){
    //RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials("");
    // Pose3d[] positions = new Pose3d[rawFiducials.length];
    // for(int i = 0; i<rawFiducials.length; i++){
    //   int currentID = rawFiducials[i].id;
    //   positions[i] = new Pose3d(Constants.OperatorConstants.aprilTagX[currentID]/39.37, Constants.OperatorConstants.aprilTagY[currentID]/39.37, Constants.OperatorConstants.aprilTagZ[currentID]/39.37, new Rotation3d(0, 0, Constants.OperatorConstants.aprilTagYaw[currentID]) );
    // }

    Pose3d[] positions = new Pose3d[Constants.OperatorConstants.aprilTagX.length];

    for(int i = 0; i<Constants.OperatorConstants.aprilTagX.length; i++){
      positions[i] = new Pose3d(Constants.OperatorConstants.aprilTagX[i]/39.37, Constants.OperatorConstants.aprilTagY[i]/39.37, Constants.OperatorConstants.aprilTagZ[i]/39.37, new Rotation3d(0, 0, Constants.OperatorConstants.aprilTagYaw[i]) );
    }
    
    
    return positions;
  }

  

  public double getAlgaeDistance() {
    //Hyperbolic regression to determine distance from algae
    return 259.2271/(LimelightHelpers.getTA("") + 1.5577);
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}