// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();
  private double tagID = -1;

  public AlignToReefTagRelative(CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(Constants.OperatorConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.OperatorConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.OperatorConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.OperatorConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.OperatorConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.OperatorConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.OperatorConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(Constants.OperatorConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.OperatorConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);

      double xSpeed = -xController.calculate(positions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      drivebase.setControl(robotDrive
      .withVelocityX(xSpeed)
      .withVelocityY(ySpeed)
      .withRotationalRate(rotValue)
      );

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
        drivebase.setControl(robotDrive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
        );
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(robotDrive
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0)
      );
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.OperatorConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.OperatorConstants.POSE_VALIDATION_TIME);
  }
}