package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

    // TODO: Ensure this matches your drivetrain's max speed (Meters/Second)
    private final double MAX_SPEED = 4.5; 

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public AutoLock(CommandSwerveDrivetrain drivetrain, Pose2d autoLockTarget, Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
        this.drivetrain = drivetrain;
        this.autoLockTarget = autoLockTarget;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        // PID Tuning Note:
        // Since we convert the output to Radians below, a Kp of 4 on Degrees might be slow.
        // 10 deg error * 4 = 40. 40 degrees converted to radians is only ~0.7 rad/s.
        // You might need to increase Kp significantly (try 8-12) or remove the unit conversion.
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
            // Multiplied by MAX_SPEED assuming suppliers return -1 to 1
            .withVelocityX(xSupplier.get() * MAX_SPEED) 
            .withVelocityY(ySupplier.get() * MAX_SPEED) 
            .withRotationalRate(Units.degreesToRadians(rotateSpeed)) 
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
            // Fallback: aim directly at target
            double dx = lockTarget.getX() - robotPose.getX();
            double dy = lockTarget.getY() - robotPose.getY();
            return Units.radiansToDegrees(Math.atan2(dy, dx));
        }

        // --- TIME IN AIR ---
        double horizontalVelocity = launcherVelocity * Math.cos(angleRad);
        double timeInAir = 0;
        
        if(horizontalVelocity > 0) {
             timeInAir = distance / horizontalVelocity;
        }

        // --- MOMENTUM COMPENSATION ---
        // 1. Get Field Relative Speeds (Robot speeds projected onto the field)
        ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, 
            drivetrain.getState().Pose.getRotation()
        );

        // 2. Calculate the offset based on real velocity (not joystick input)
        // We use a small deadband (0.1) just to stop jitter when stopped
        double offsetX = 0;
        double offsetY = 0;

        if(Math.abs(fieldRelSpeeds.vxMetersPerSecond) > 0.1) {
            offsetX = fieldRelSpeeds.vxMetersPerSecond * timeInAir;
        }
        if(Math.abs(fieldRelSpeeds.vyMetersPerSecond) > 0.1) {
            offsetY = fieldRelSpeeds.vyMetersPerSecond * timeInAir;
        }

        // 3. VIRTUAL TARGET CALCULATION
        // FIX: We SUBTRACT the offset. 
        // If we are moving East (+X), the ball carries East momentum.
        // To hit the center, we must aim West (-X) relative to the target.
        double virtualTargetX = lockTarget.getX() - offsetX;
        double virtualTargetY = lockTarget.getY() - offsetY;

        // 4. Calculate angle to Virtual Target
        double dx = virtualTargetX - robotPose.getX();
        double dy = virtualTargetY - robotPose.getY();

        return Units.radiansToDegrees(Math.atan2(dy, dx));
    }
}