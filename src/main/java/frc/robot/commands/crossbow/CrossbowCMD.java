package frc.robot.commands.crossbow;

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
        // high variable allows two buttons to use the same command but set the crossbow to two different positions
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
