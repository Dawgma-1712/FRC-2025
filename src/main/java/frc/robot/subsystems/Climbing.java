package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.*;
import frc.Constants.OperatorConstants;

public class Climbing extends SubsystemBase{
    private final TalonFX climbingMotor1 = new TalonFX(OperatorConstants.climbingMotorID1);
    private final TalonFX climbingMotor2 = new TalonFX(OperatorConstants.climbingMotorID2);

    private Angle position1, position2; 
    private double desiredPosition = -62;
    //private double desiredPosition = -30;

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

    public void setSetpoint(double setpoint) {
        desiredPosition = setpoint;
    }

    public double getSetpoint() {
        return desiredPosition;
    }

    public void setSpeed(double speed) {
        climbingMotor1.set(speed);
        climbingMotor2.set(-speed); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber", climbingMotor1.getPosition().getValueAsDouble());
        // double climberDifference = climbingMotor1.getPosition().getValueAsDouble() - desiredPosition;

        // if(Math.abs(climberDifference) < OperatorConstants.crossbowMarginOfError) {
        //     stop();
        // } else {
        //     setSpeed(climberDifference > 0 ? -0.1 : 0.1);
        // }
    }
    
    public double getPosition(){
        return position1.magnitude();
    }
}
