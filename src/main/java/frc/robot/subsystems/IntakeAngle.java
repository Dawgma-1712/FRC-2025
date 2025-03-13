package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.OperatorConstants;

public class IntakeAngle extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(OperatorConstants.intakeAngleID);
    private double desiredPosition = 0;
    private DigitalInput bottomSwitch = new DigitalInput(2);
    private DigitalInput topSwitch = new DigitalInput(1);

    public IntakeAngle() {
        zeroPosition();
    }

    public void stop() {
        armMotor.setControl(new MotionMagicVoltage(0));
    }

    public void setPosition(double position) {
        armMotor.setPosition(position / 3.6);
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

    @Override
    public void periodic() {
          if(!topSwitch.get()) {
              setPosition(OperatorConstants.topSwitchPosition);
         }

         if(desiredPosition <= OperatorConstants.topSwitchPosition) {
             desiredPosition = OperatorConstants.topSwitchPosition;
         }

         else if(desiredPosition >= OperatorConstants.bottomSwitchPosition) {
             desiredPosition = OperatorConstants.bottomSwitchPosition;
         }

         SmartDashboard.putNumber("Intake Angle", getPosition());

        PIDPosition(desiredPosition);
    }
}