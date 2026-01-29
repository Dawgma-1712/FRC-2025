package frc.robot.commands.intake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.Constants.OperatorConstants;

public class SetIntakeAngleCMD extends Command {
    private IntakeAngle intake; 
    private double setpoint;

    public SetIntakeAngleCMD(IntakeAngle intake, double setpoint) {
        this.intake = intake; 
        this.setpoint = setpoint;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // ends command if intake is within 2 degrees of target
        return Math.abs(intake.getPosition() - intake.getSetpoint()) <= 2; 
    }
}
