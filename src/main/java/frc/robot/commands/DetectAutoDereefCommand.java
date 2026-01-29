package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Import your Drivetrain
import java.util.Set; // Needed for Commands.defer

public class DetectAutoDereefCommand extends Command {
    private final RobotContainer m_robotContainer;
    private final CommandSwerveDrivetrain m_drivetrain;

    // Pass the RobotContainer and Drivetrain into the constructor
    public DetectAutoDereefCommand(RobotContainer robotContainer, CommandSwerveDrivetrain drivetrain) {
        m_robotContainer = robotContainer;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain); // Add any subsystems that the auto-dereef uses
    }

    @Override
    public void execute() {
        // 1. Check if the global variable is true
        if (RobotContainer.runAutoDereef) {
            // 2. Schedule the command returned by getAutoDereef
            // This schedules the actual pathfinding command
            Command autoCommand = m_robotContainer.getAutoDereef("Reef", () -> m_drivetrain.getState().Pose);
            if (autoCommand != null) {
                autoCommand.schedule();
            }
            
            // 3. Set the global variable back to false
            RobotContainer.runAutoDereef = false;
        }
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously
        return false;
    }
    
    // Commands that run continuously should generally not end and should only require subsystems
}