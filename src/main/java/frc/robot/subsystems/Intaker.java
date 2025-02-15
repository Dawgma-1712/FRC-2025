package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.OperatorConstants;;

public class Intaker extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(OperatorConstants.intakeMotorID, MotorType.kBrushed);
    
    public Intaker() {}

    public void stop() {
        intakeMotor.set(0);
    }

    public void setMotor(double motorSpeed) {
        intakeMotor.set(motorSpeed);
    }

    
}
