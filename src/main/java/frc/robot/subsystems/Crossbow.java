package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.Constants.OperatorConstants;


public class Crossbow extends SubsystemBase {
    
    private final SparkMax crossbowMotor = new SparkMax(OperatorConstants.crossbowMotorID, MotorType.kBrushless);
    private DigitalInput crossbowLimitSwitch = new DigitalInput(OperatorConstants.crossbowLimitSwitchID);
    private DigitalInput crossbowMagLimSwitch = new DigitalInput(OperatorConstants.crossbowMagLimitSwitchID);

    // in rotations
    private double desiredPosition;

    public Crossbow() {
        crossbowMotor.getEncoder().setPosition(0);
        desiredPosition = 0;
    }

    @Override
    public void periodic() {
        //High position should be 8300

        double crossbowDifference = crossbowMotor.getEncoder().getPosition()*360.0 - desiredPosition;

        if(Math.abs(crossbowDifference) < OperatorConstants.crossbowMarginOfError) {
            stop();
        } else {
            crossbowMotor.set(crossbowDifference > 0 ? OperatorConstants.crossbowSpeed : -OperatorConstants.crossbowSpeed);
        }

        if(!crossbowLimitSwitch.get()) {
            crossbowMotor.getEncoder().setPosition(0);
        }

        if(desiredPosition < 0) {
            desiredPosition = 5;
        }

    }

    public void stop() {
        crossbowMotor.set(0);
    }

    public void setSpeed(double motorSpeed) {
        crossbowMotor.set(motorSpeed);
        desiredPosition = crossbowMotor.getEncoder().getPosition()*360.0;
    }

    public void highCrossbow() {
        desiredPosition = OperatorConstants.topCrossbow;
    }

    public void lowCrossbow() {
        desiredPosition = OperatorConstants.bottomCrossbow;
    }

}
