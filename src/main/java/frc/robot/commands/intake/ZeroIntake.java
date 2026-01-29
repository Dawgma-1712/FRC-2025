package frc.robot.commands.intake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroIntake extends Command {
    private IntakeAngle intake; 
    private double position;

    public ZeroIntake(IntakeAngle intake, double position) {
        this.intake = intake; 
        this.position = position;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
