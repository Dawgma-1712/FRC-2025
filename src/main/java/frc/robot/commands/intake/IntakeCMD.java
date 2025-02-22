package frc.robot.commands.intake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command; 

public class IntakeCMD extends Command {
    private Intaker intaker; 
    private double motorSpd; 

    public IntakeCMD(Intaker intaker, double motorSpd) {
        this.intaker = intaker; 
        this.motorSpd = motorSpd; 
        addRequirements(intaker);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intaker.setMotor(-motorSpd);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false; 
    }

    
}
