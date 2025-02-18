package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbing;
import frc.Constants.OperatorConstants;

public class ManualClimbing extends Command {

    private Climbing climb=new Climbing();
    boolean up;

    public ManualClimbing(Climbing climb, boolean up){
        this.climb = climb;
        this.up = up;

    }
    @Override
    public void execute(){
        if(up){
            climb.setClimber(climb.getPosition() + OperatorConstants.increaseAmount);
        } else{
            climb.setClimber(climb.getPosition() - OperatorConstants.increaseAmount);
        }
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
