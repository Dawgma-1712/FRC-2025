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

import java.lang.management.OperatingSystemMXBean;

public class Climbing extends SubsystemBase{
    private final TalonFX climbingMotor1 = new TalonFX(16);
    private final TalonFX climbingMotor2 = new TalonFX(17);

    private Angle position1, position2; 

    public Climbing() {
        this.position1 = Degree.of(0); 
        this.position2 = Degree.of(0);

    }
    public void stop(){
        climbingMotor1.set(0);
        climbingMotor2.set(0); 
    }

    public void setClimber(double angle) {
        this.position1 = Degree.of(angle);
        this.position2 = Degree.of(-angle); 
    }

    @Override
    public void periodic() {
        climbingMotor1.setPosition(position1);
        climbingMotor2.setPosition(position2);
    }
    
    public double getPosition(){
        return position1.magnitude();
    }
}
