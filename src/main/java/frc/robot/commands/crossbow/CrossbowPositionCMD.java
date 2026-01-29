package frc.robot.commands.crossbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Crossbow;

public class CrossbowPositionCMD extends Command {
    
    private Crossbow crossbow;
    private double position;

    public CrossbowPositionCMD(Crossbow crossbow, double position) {
        this.crossbow = crossbow;
        this.position = position;
        addRequirements(crossbow);
    }

    @Override
    public void initialize() {
        crossbow.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
