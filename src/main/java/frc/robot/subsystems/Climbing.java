package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.Constants.OperatorConstants;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

public class Climbing extends SubsystemBase{
    private final TalonFX climbingMotor1 = new TalonFX(16);
    private final TalonFX climbingMotor2 = new TalonFX(17);

    private Angle position = Degrees.of(0);


    public void stop(){
        climbingMotor1.set(0);
        climbingMotor2.set(0);
    }

    public void setMotors(double motorSpeed1, double motorSpeed2){
        climbingMotor1.set(motorSpeed1);
        climbingMotor2.set(motorSpeed2);
    }

}
