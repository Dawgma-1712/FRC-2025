package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.Constants;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

public class AutoAlign extends Command{

    private final double inchesMaxSpeed = 100.0;
    private CommandSwerveDrivetrain swerve;
    private Vision vision;
    private double xGoal, yGoal, yawGoal;
    private Double[] ids = Constants.OperatorConstants.id_Blue;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoAlign(CommandSwerveDrivetrain swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

        if(vision.getTID() != 0) {
            xGoal = Constants.OperatorConstants.aprilTagX[(int)(vision.getTID()) - 1];
            yGoal = Constants.OperatorConstants.aprilTagY[(int)(vision.getTID()) - 1];
            yawGoal = Constants.OperatorConstants.aprilTagYaw[(int)(vision.getTID()) - 1];
        }
        
        else {
            xGoal = 0;
            yGoal = 0;
            yawGoal = 0;
        }

        addRequirements(swerve);
    }

    public void initialize() {
    }

    public void execute() {
        if(Arrays.asList(ids).contains(vision.getTID())) {
            double xDiff = xGoal - vision.getFieldX();
            double yDiff = yGoal - vision.getFieldY();
            double yawDiff = yawGoal - vision.getFieldYaw();
    
            //If it's too slow, try logarithmic function
            swerve.applyRequest(() -> drive.withVelocityX(clamp(xDiff/inchesMaxSpeed, -1, 1))
                .withVelocityY(clamp(yDiff/inchesMaxSpeed, -1, 1))
                .withRotationalRate(clamp(yawDiff/inchesMaxSpeed, -1, 1))
                );
        }
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return (Math.abs(vision.getFieldX() - xGoal) < 1
        && Math.abs(vision.getFieldY() - yGoal) < 1
        && Math.abs(vision.getFieldYaw() - yawGoal) < 1) || vision.getTA() <= 0;
    }

    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}