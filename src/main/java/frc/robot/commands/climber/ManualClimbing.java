package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbing;
import frc.Constants.OperatorConstants;

public class ManualClimbing extends Command {

    private Climbing climb=new Climbing();
    private boolean up;
    private double speed;

    public ManualClimbing(Climbing climb, boolean up, double speed){
        this.climb = climb;
        this.up = up;
        this.speed = speed;
        addRequirements(climb);
    }
    @Override
    public void execute(){
        // this command is bound to two buttons, one that makes it go "up" and another that makes it go "down"
        // up variable allows command to be used for both directions
        // if(up) {
        //      climb.setClimber(climb.getPosition() + OperatorConstants.increaseAmount);
        //  } else {
        //      climb.setClimber(climb.getPosition() - OperatorConstants.increaseAmount);
        //  }

        System.out.println("Manually climbing");
        climb.setSpeed(speed);
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
