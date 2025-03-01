package frc.robot.commands.intake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.Constants.OperatorConstants;

public class IntakeAngleCMD extends Command {
    private IntakeAngle intake; 
    private boolean stowed = true;

    public IntakeAngleCMD(IntakeAngle intake) {
        this.intake = intake; 
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(stowed)
            intake.setSetpoint(OperatorConstants.intakeAngle);
        else
            intake.setSetpoint(OperatorConstants.stowAngle);
    }

    @Override
    public void end(boolean interrupted) {
        stowed = !stowed;
    }

    @Override
    public boolean isFinished() {
        // ends command if intake is within 2 degrees of target
        return Math.abs(intake.getPosition() - intake.getSetpoint()) <= 2; 
    }
}
