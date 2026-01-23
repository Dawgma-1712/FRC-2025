package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoLock extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotationController;
    private final Pose2d autoLockTarget;
    
    // Joystick Suppliers
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public AutoLock(CommandSwerveDrivetrain drivetrain, Pose2d autoLockTarget, Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
        this.drivetrain = drivetrain;
        this.autoLockTarget = autoLockTarget;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        // Tune these PID values carefully. 
        // If output is meant to be Rad/s for Phoenix 6, ensure Kp is sized appropriately.
        this.rotationController = new PIDController(4, 0, 0.02);
        
        this.rotationController.enableContinuousInput(-180, 180); 
        
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
        double goalAngle = findAutoLockAngle(autoLockTarget);

        double rotateSpeed = rotationController.calculate(currentHeading, goalAngle);

        drivetrain.setControl(driveRequest
            // WARNING: Ensure these suppliers return Meters/Second, not just -1 to 1!
            .withVelocityX(xSupplier.get()) 
            .withVelocityY(ySupplier.get()) 
            .withRotationalRate(Units.degreesToRadians(rotateSpeed)) // Convert PID output (likely degrees) to Radians/Sec for Phoenix 6? Check your tuning units.
        );
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) { 
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    private double findAutoLockAngle(Pose2d lockTarget) {
        Pose2d robotPose = drivetrain.getState().Pose;
        double distance = robotPose.getTranslation().getDistance(lockTarget.getTranslation());

        double angle = Constants.OperatorConstants.launcherAngle;
        double heightDif = Constants.OperatorConstants.hubHeight - Constants.OperatorConstants.launcherHeight;
        double g = 9.81;
        double angleRad = Math.toRadians(angle);
        
        double launcherVelocity = 0.0;
        
        // --- PHYSICS CALCULATION ---
        if (Math.abs(angle) != 90 && (distance * Math.tan(angleRad) > heightDif)) {
            launcherVelocity = Math.sqrt((g * Math.pow(distance, 2)) / (2 * Math.pow(Math.cos(angleRad), 2) * (distance * Math.tan(angleRad) - heightDif)));
        } else {
            // If shot is impossible, just look directly at target (don't compensate)
            // or aim at a default angle to avoid NaN errors
            double dx = lockTarget.getX() - robotPose.getX();
            double dy = lockTarget.getY() - robotPose.getY();
            return Units.radiansToDegrees(Math.atan2(dy, dx));
        }

        // --- TIME IN AIR FIX ---
        // Formula: t = d / (v * cos(theta))
        double horizontalVelocity = launcherVelocity * Math.cos(angleRad);
        double timeInAir = 0;
        
        // Prevent divide by zero if something went wrong
        if(horizontalVelocity > 0) {
             timeInAir = distance / horizontalVelocity;
        }

        // --- MOMENTUM COMPENSATION ---
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

        // FIXED: Removed "if > 0.1" to allow negative velocity compensation
        double offsetX = 0, offsetY = 0;

        if(Math.abs(chassisSpeeds.vxMetersPerSecond)>0.25 && (xSupplier.get()!=0)){
            offsetX=timeInAir*chassisSpeeds.vxMetersPerSecond;
        }
        if(Math.abs(chassisSpeeds.vyMetersPerSecond)>0.25 && (ySupplier.get()!=0)){
            offsetY=timeInAir*chassisSpeeds.vyMetersPerSecond;
        }


        // Calculate "Virtual Target" position by subtracting robot motion
        double distanceX = lockTarget.getX() - robotPose.getX() + offsetX;
        double distanceY = lockTarget.getY() - robotPose.getY() + offsetY;

        double angleRadRatio = Math.atan2(distanceY, distanceX);
        return Units.radiansToDegrees(angleRadRatio);
    }

    // private double getFieldOffsetX() {
    //     ChassisSpeeds chassisSpeeds = drivetrain.getState().Speeds;
    //     double fieldX = chassisSpeeds.
    // }
}