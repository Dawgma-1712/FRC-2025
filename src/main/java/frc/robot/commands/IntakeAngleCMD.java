package frc.robot.commands;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.Units.*;

public class IntakeAngleCMD extends Command {
    private IntakeAngle intake; 
    private boolean stowed = true;

    public IntakeAngleCMD(IntakeAngle intake) {
        this.intake = intake; 
    }

    public void initialize() {
        if(stowed)
            intake.setStow();
        else
            intake.setIntake();
        stowed = !stowed;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false; 
    }

    
}
