package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Crossbow;

public class CrossbowCMD extends Command {
    
    private Crossbow crossbow;
    private boolean high;

    public CrossbowCMD(Crossbow crossbow, boolean high) {
        this.crossbow = crossbow;
        this.high = high;
        addRequirements(crossbow);
    }

    @Override
    public void initialize() {
        if(high) {
            crossbow.highCrossbow();
        } else {
            crossbow.lowCrossbow();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
