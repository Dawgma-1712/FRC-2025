package frc.robot.commands;

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
        //intakeAngle.setPosition(intakeAngle.getPosition().magnitude() + motorSpeed.get());
        //intakeAngle.moveToPosition(intakeAngle.getPosition() + motorSpeed.get() * 0.1);

        // System.out.println(intakeAngle.getPosition());
        // intakeAngle.setSpeed(motorSpeed.get());

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
