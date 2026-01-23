package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command
;
import frc.robot.subsystems.Shooter;

import frc.Constants;

public class ShooterCMD extends Command {
    private Shooter shooter; 
    private double motorSpd; 

    public ShooterCMD(Shooter shooter, double motorSpd) {
        this.shooter = shooter; 
        this.motorSpd = motorSpd; 
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
            shooter.setMotor(-motorSpd);
        }

    public double findOptimalVelocity(double distance){
        double angle = Constants.OperatorConstants.launcherAngle;

        double heightDif = Constants.OperatorConstants.hubHeight-Constants.OperatorConstants.launcherHeight;



        double velocity = Math.sqrt((9.8*distance*distance)/(2*Math.cos(angle)*Math.cos(angle) * ((distance * Math.tan (angle)) - heightDif)));

        return velocity;



    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false; 
    }

    
}
