package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngle;
import java.util.function.Supplier;

public class ManualAngleCMD extends Command {

    IntakeAngle intakeAngle;
    Supplier<Double> motorSpeed;

    public ManualAngleCMD(IntakeAngle intakeAngle, Supplier<Double> motorSpeed) {
        this.intakeAngle = intakeAngle;
        this.motorSpeed = motorSpeed;
        addRequirements(intakeAngle);
    }

    @Override
    public void execute() {
        intakeAngle.setPosition(intakeAngle.getPosition().magnitude() + motorSpeed.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
