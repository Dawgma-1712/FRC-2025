package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.Constants.OperatorConstants;

public class Crossbow extends SubsystemBase {
    
    private final SparkMax crossbowMotor = new SparkMax(15, MotorType.kBrushless);

    // in rotations
    private double desiredPosition;

    public Crossbow() {
        desiredPosition = 0;
    }

    @Override
    public void periodic() {

        double crossbowDifference = crossbowMotor.getEncoder().getPosition() - desiredPosition;

        if(Math.abs(crossbowDifference) < OperatorConstants.crossbowMarginOfError) {
            stop();
        } else {
            crossbowMotor.set(crossbowDifference > 0 ? OperatorConstants.crossbowSpeed : -OperatorConstants.crossbowSpeed);
        }

    }

    public void stop() {
        crossbowMotor.set(0);
    }

    public void setSpeed(double motorSpeed) {
        crossbowMotor.set(motorSpeed);
    }

    public void highCrossbow() {
        desiredPosition = OperatorConstants.topCrossbow;
    }

    public void lowCrossbow() {
        desiredPosition = OperatorConstants.bottomCrossbow;
    }

}
