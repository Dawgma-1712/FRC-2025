package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.Constants.OperatorConstants;
import edu.wpi.first.units.measure.Angle;

public class IntakeAngle extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(25);
    private Angle position = Degrees.of(0);
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

    public IntakeAngle() {
        this.position = Degrees.of(0);
    }

    public void stop() {
        armMotor.setControl(new MotionMagicVoltage(0));
    }

    public void setPosition(double position) {
        this.position = Degrees.of(position);
    }

    public void zeroPosition() {
        armMotor.setPosition(0);
    }

    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble() * 3.6 + 10.7;
    }

    public void moveToPosition(double rotations) {
        armMotor.setControl(motionMagicControl.withPosition(rotations));
    }

    public double getSpeed() {
        return armMotor.get();
    }

    public void setStow() {
        this.position = Degrees.of(OperatorConstants.stowAngle);
    }

    public void setSpeed(double speed) {
        DutyCycleOut dCycleOut = new DutyCycleOut(speed * 0.3);
        armMotor.setControl(dCycleOut);
    }

    public void setIntake() {
        this.position = Degrees.of(OperatorConstants.intakeAngle);
    }

    @Override
    public void periodic() {
        //intakeAngleMotor.setPosition(position);
    }
}
