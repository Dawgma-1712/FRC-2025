package frc.robot.commands;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngle;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
        System.out.println("Executing1");
        //intakeAngle.moveToPosition(intakeAngle.getPosition() + motorSpeed.get() * 0.1);

        intakeAngle.setSpeed(motorSpeed.get());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        System.out.print("Done");
        return false;
    }

}
