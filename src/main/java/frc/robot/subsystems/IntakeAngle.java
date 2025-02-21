package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.Constants.OperatorConstants;
import edu.wpi.first.units.measure.Angle;

public class IntakeAngle extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(25);
    private double desiredPosition = 0;

    public IntakeAngle() {
    }

    public void stop() {
        armMotor.setControl(new MotionMagicVoltage(0));
    }

    public void setPosition(double position) {
        armMotor.setPosition(position);
    }

    public void zeroPosition() {
        armMotor.setPosition(0);
    }

    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble() * 3.6;
    }

    public void PIDPosition(double position) {
        //0 is straight up
        PositionVoltage pVoltage = new PositionVoltage(0).withSlot(0);
        armMotor.setControl(pVoltage.withPosition(position / 3.6).withFeedForward(0));
    }

    public void moveToPosition(double position) {
        desiredPosition = position;
    }

    public double getSpeed() {
        return armMotor.get();
    }

    public double getSetpoint() {
        return desiredPosition;
    }

    public void setSetpoint(double setpoint) {
        desiredPosition = setpoint;
    }

    public void setSpeed(double speed) {
        DutyCycleOut dCycleOut = new DutyCycleOut(speed * 0.3);
        armMotor.setControl(dCycleOut);
    }

    @Override
    public void periodic() {
        PIDPosition(desiredPosition);

        System.out.println();
        System.out.print(getPosition() + "a");
        System.out.println();
    }
}