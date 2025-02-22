package frc.robot.commands.climber;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbingCMD extends Command {
    private Climbing climbing; 
    private double angle; 

    public ClimbingCMD(Climbing climbing, double angle) {
        this.climbing = climbing;
        this.angle = angle;
        addRequirements(climbing); 
    }

    public void initialize() {}

    @Override
    public void execute() {
        climbing.setClimber(angle * 45);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true; 
    }


} 