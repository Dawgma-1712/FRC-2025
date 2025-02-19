package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.Constants.OperatorConstants;
import edu.wpi.first.units.measure.Angle;

public class IntakeAngle extends SubsystemBase {
    private final TalonFX intakeAngleMotor = new TalonFX(13);
    private Angle position;
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    Orchestra m_orchestra = new Orchestra();

    public IntakeAngle() {
        this.position = Degrees.of(0);

        // Add a single device to the orchestra
        m_orchestra.addInstrument(intakeAngleMotor);

        // Attempt to load the chrp
        var status = m_orchestra.loadMusic("output.chrp");
        m_orchestra.play();
    }

    public void stop() {
        intakeAngleMotor.set(0);
    }

    public void setPosition(double position) {
        this.position = Degrees.of(position);
    }

    public Angle getPosition() {
        return position;
    }

    public void setStow() {
        this.position = Degrees.of(OperatorConstants.stowAngle);
    }

    public void setSpeed(double speed) {
        driveDutyCycle.Output = speed;
            intakeAngleMotor.setControl(driveDutyCycle);
    }

    public void setIntake() {
        this.position = Degrees.of(OperatorConstants.intakeAngle);
    }

    @Override
    public void periodic() {
        //intakeAngleMotor.setPosition(position);
    }
}
