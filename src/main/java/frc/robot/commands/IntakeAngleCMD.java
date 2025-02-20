// package frc.robot.commands;
// import frc.robot.subsystems.*;
// import edu.wpi.first.wpilibj2.command.Command; 

// public class IntakeAngleCMD extends Command {
//     private IntakeAngle intake; 
//     private boolean stowed = true;

//     public IntakeAngleCMD(IntakeAngle intake) {
//         this.intake = intake; 
//         addRequirements(intake);
//     }

//     @Override
//     public void initialize() {
//         if(stowed)
//             intake.setStow();
//         else
//             intake.setIntake();
//         stowed = !stowed;
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {}

//     @Override
//     public boolean isFinished() {
//         return true; 
//     }

    
// }
