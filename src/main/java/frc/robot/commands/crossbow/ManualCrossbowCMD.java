package frc.robot.commands.crossbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Crossbow;
import java.util.function.Supplier;

public class ManualCrossbowCMD extends Command {
    // but what about third crossbow command?
    private Crossbow crossbow;
    private Supplier<Double> crossbowSpeed;
    
    public ManualCrossbowCMD(Crossbow crossbow, Supplier<Double> crossbowSpeed) {
        this.crossbow = crossbow;
        this.crossbowSpeed  = crossbowSpeed;
        addRequirements(crossbow);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        crossbow.setSpeed(crossbowSpeed.get() * 0.5);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
