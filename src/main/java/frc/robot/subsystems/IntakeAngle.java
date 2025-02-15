package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.OperatorConstants;
import edu.wpi.first.units.measure.Angle;

public class IntakeAngle extends SubsystemBase {
    private final TalonFX intakeAngleMotor = new TalonFX(13);
    private Angle position;

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

    public void setStow() {
        this.position = Degrees.of(OperatorConstants.stowAngle);
    }

    public void setIntake() {
        this.position = Degrees.of(OperatorConstants.intakeAngle);
    }

    @Override
    public void periodic() {
        intakeAngleMotor.setPosition(position);
    }
}
