package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.OperatorConstants;;

public class Shooter extends SubsystemBase {
    int shooterMotorID = 3;
    private final SparkMax shootMotor = new SparkMax(shooterMotorID, MotorType.kBrushed);

    
    public Shooter() {

    }

    public void stop() {
        shootMotor.set(0);
    }


    public void setMotor(double motorSpeed) {
        shootMotor.set(motorSpeed);
    }

}
