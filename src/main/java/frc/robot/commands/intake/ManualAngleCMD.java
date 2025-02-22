package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngle;
import java.util.function.Supplier;

public class ManualAngleCMD extends Command {

    private IntakeAngle intakeAngle;
    private Supplier<Double> motorSpeed;

    public ManualAngleCMD(IntakeAngle intakeAngle, Supplier<Double> motorSpeed) {
        this.intakeAngle = intakeAngle;
        this.motorSpeed = motorSpeed;
        addRequirements(intakeAngle);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // updates position that the intake is trying to go to by whatever the joystick is at
        intakeAngle.setSetpoint(intakeAngle.getPosition() + motorSpeed.get() * 10);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
