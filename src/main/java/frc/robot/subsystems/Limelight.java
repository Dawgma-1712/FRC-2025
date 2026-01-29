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
import edu.wpi.first.math.geometry.Transform3d;
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
import frc.Constants.OperatorConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.ArrayList;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = true;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private static final RectanglePoseArea field = new RectanglePoseArea(new Translation2d(0.0, 0.0),
      new Translation2d(16.54, 8.02));

  StructPublisher<Pose3d> publisher3D =
  NetworkTableInstance.getDefault().getStructTopic("AprilTag", Pose3d.struct).publish();

  StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("AprilTagArray", Pose3d.struct).publish();

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

  @Override
  public void periodic() {
    if (enable) {
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation()
          .getDistance(new Translation3d());
      Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.LimelightResults result = LimelightHelpers.getLatestResults(ll);
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
    }
  }

  @Override
  public void simulationPeriodic() {
    arrayPublisher.set(aprilTagsVisible());
  }

  // returns whether an apriltag is visible based on the camera's field of view
  // and the location of the tag
  public boolean isAprilTagVisible(Pose3d limelightPose, Pose3d tagPose) {
    
    // where the tag is relative to the limelight, where the limelight is (0, 0, 0) and the direction it points is the positive x-axis
    Pose3d tagToLimelight = tagPose.relativeTo(limelightPose);
    
    // calculates 2D distance and considers the apriltag visible only if it's within a certain range
    double distance = Math.sqrt(
        tagToLimelight.getX() * tagToLimelight.getX() +
            tagToLimelight.getY() * tagToLimelight.getY());

    if (distance >= OperatorConstants.LIMELIGHT_RANGE)
      return false;

    double tagPoseRotationDegrees = Math.toDegrees(tagPose.getRotation().getAngle());
    double limelightPoseRotationDegrees = Math.toDegrees(limelightPose.getRotation().getAngle());

    if (tagPoseRotationDegrees > 180)
      tagPoseRotationDegrees -= 180;
    else
      tagPoseRotationDegrees += 180;

    if (Math.abs(tagPoseRotationDegrees - limelightPoseRotationDegrees) > 60) {
      return false;
    }

    // calculates whether the apriltag is within the limelight's fov
    double yaw = Math.atan2(tagToLimelight.getY(), tagToLimelight.getX());
    double pitch = Math.atan2(tagToLimelight.getZ(), tagToLimelight.getX());

    double horizontal_fov_radians = Math.toRadians(OperatorConstants.LIMELIGHT_HORIZONTAL_FOV);
    double vertical_fov_radians = Math.toRadians(OperatorConstants.LIMELIGHT_VERTICAL_FOV);

    boolean inFov = Math.abs(yaw) < horizontal_fov_radians / 2 &&
        Math.abs(pitch) < vertical_fov_radians / 2;

    return inFov;
  }

  public Pose3d[] aprilTagsVisible() {

    Pose3d robotPose = new Pose3d(drivetrain.getState().Pose);
    Transform3d robotToCamera = OperatorConstants.LIMELIGHT_TO_ROBOT;
    Pose3d limelightPose = robotPose.transformBy(robotToCamera);

    ArrayList<Pose3d> positions = new ArrayList<Pose3d>();

    for (int i = 1; i <= Constants.OperatorConstants.aprilTagX.length; i++) {

      Pose3d tagPose = OperatorConstants.APRIL_TAG_POSES.getTagPose(i).orElse(new Pose3d());

      if (isAprilTagVisible(limelightPose, tagPose)) {
        positions.add(tagPose);
      }

    }

    Pose3d[] positions_array = positions.toArray(new Pose3d[0]);
    SmartDashboard.putNumber("Apriltags Seen", positions_array.length);
    return positions_array;
  }

  public double getAlgaeDistance() {
    // Hyperbolic regression to determine distance from algae
    return 259.2271 / (LimelightHelpers.getTA("") + 1.5577);
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