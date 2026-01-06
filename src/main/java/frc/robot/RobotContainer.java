// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// auto imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.commands.climber.ManualClimbing;
import frc.robot.commands.intake.ManualAngleCMD;
import frc.robot.commands.intake.SetIntakeAngleCMD;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.intake.IntakeAngleCMD;
import frc.robot.commands.intake.IntakeCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.Constants;
import frc.Constants.OperatorConstants;
import frc.robot.commands.crossbow.*;

public class RobotContainer {
    public static double speed = 1;

    public static boolean runAutoDereef = false;


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Limelight limelight = new Limelight(drivetrain);
    
    // subsystems
    
    public final Intaker intaker = new Intaker();
    public final IntakeAngle intakeAngle = new IntakeAngle();
    public final Crossbow crossbow = new Crossbow();
    public final Climbing climbing = new Climbing();

    private final Joystick driver = new Joystick(0); 
    private final Joystick operator = new Joystick(1);

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> pathChooser;

    Pose2d targetPose = new Pose2d(12.4, 5.82, Rotation2d.fromDegrees(-57));
    //If the robot's rotation doesn't match what it is told, it breaks the pathfinder stuff
    

    PathConstraints constraints = new PathConstraints(.3, .4, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public RobotContainer() {
        SmartDashboard.putNumber("Wait Time", 0);

        Command autoDereefL2Command;
        autoDereefL2Command = new SequentialCommandGroup(new SetIntakeAngleCMD(intakeAngle, OperatorConstants.dereefAngle), new WaitCommand(0.5), new SetIntakeAngleCMD(intakeAngle, OperatorConstants.stowAngle + 10), new WaitCommand(0.5));
        Command intakeCommand = new SequentialCommandGroup(new IntakeCMD(intaker, -0.6).raceWith(new WaitCommand(1.5)), new IntakeCMD(intaker, 0));
        NamedCommands.registerCommand("L2Dereef", autoDereefL2Command.raceWith(intakeCommand));

        Command coralClipCommand;
        coralClipCommand = new SequentialCommandGroup(new SetIntakeAngleCMD(intakeAngle, 40), new WaitCommand(0.5));
        NamedCommands.registerCommand("CoralClip", coralClipCommand);

        Command autoScoreCommand;
        autoScoreCommand = new SequentialCommandGroup(new SetIntakeAngleCMD(intakeAngle, OperatorConstants.stowAngle - 10), new WaitCommand(1));
        NamedCommands.registerCommand("Score", autoScoreCommand.raceWith(new IntakeCMD(intaker, -0.6)));

        Command autoDereefL3Command;
        autoDereefL3Command = new SequentialCommandGroup(new WaitCommand(0.5), new CrossbowPositionCMD(crossbow, 25.5), new SetIntakeAngleCMD(intakeAngle, 30), new WaitCommand(0.5));
        NamedCommands.registerCommand("L3Dereef", autoDereefL3Command);

        Command autoRetractCommand;
        autoRetractCommand = new SequentialCommandGroup(new WaitCommand(0.5), new CrossbowPositionCMD(crossbow, 0), new WaitCommand(0.5));
        NamedCommands.registerCommand("RetractCrossbow", autoRetractCommand);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        pathChooser = new SendableChooser<>();


        pathChooser.addOption("CMAlgae", getAutoDereef("CMAlgae", () -> drivetrain.getState().Pose));
        pathChooser.addOption("CRAlgae", getAutoDereef("CRAlgae", () -> drivetrain.getState().Pose));
        pathChooser.addOption("CLAlgae", getAutoDereef("CLAlgae", () -> drivetrain.getState().Pose));
        pathChooser.addOption("FMAlgae", getAutoDereef("FMAlgae", () -> drivetrain.getState().Pose));
        pathChooser.addOption("FRAlgae", getAutoDereef("FRAlgae", () -> drivetrain.getState().Pose));
        pathChooser.addOption("FLAlgae", getAutoDereef("FLAlgae", () -> drivetrain.getState().Pose));
        pathChooser.addOption("Processor", getAutoDereef("Processor", () -> drivetrain.getState().Pose));


        pathChooser.setDefaultOption("Nearest Reef", getAutoDereef("Reef", () -> drivetrain.getState().Pose));
//we made a thing
        // Command pathToPoseDereef=new SequentialCommandGroup(getAutoDereef("Reef", () -> drivetrain.getState().Pose),autoDereefL2Command);


        SmartDashboard.putData("Path Chooser", pathChooser);


        new DetectAutoDereefCommand(this, drivetrain).schedule();
        


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(Math.abs(-driver.getRawAxis(1)) > 0.2 ? -driver.getRawAxis(1) * MaxSpeed * speed : 0) // Drive forward with negative Y (forward)
            .withVelocityY(Math.abs(-driver.getRawAxis(0)) > 0.2 ? -driver.getRawAxis(0) * MaxSpeed * speed : 0) // Drive left with negative X (left)
            .withRotationalRate(Math.abs(-driver.getRawAxis(2) * MaxAngularRate) > 0.05 ? -driver.getRawAxis(2) * MaxAngularRate : 0)
            )
        );

        intakeAngle.setDefaultCommand(new ManualAngleCMD(intakeAngle,
            () -> operator.getRawAxis(1)
        ));

        crossbow.setDefaultCommand(new ManualCrossbowCMD(crossbow,
            () -> -operator.getRawAxis(5)
        ));

        configureBindings();
    }

    private void configureBindings() {
        

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        new JoystickButton(operator, 4).onTrue(new IntakeCMD(intaker, 0.6)).onFalse(new IntakeCMD(intaker, 0)); 
        new JoystickButton(operator, 1).onTrue(new IntakeCMD(intaker, -0.6)).onFalse(new IntakeCMD(intaker, 0)); 
        
        new JoystickButton(driver,
         3).toggleOnTrue(new IntakeAngleCMD(intakeAngle));
        //new JoystickButton(driver, 3).toggleOnTrue(new CrossbowCMD(crossbow, true));
        //new JoystickButton(driver, 2).toggleOnTrue(new CrossbowCMD(crossbow, false));

        // new JoystickButton(operator, 2).onTrue(new ManualClimbing(climbing, false, 0.1)).onFalse(new ManualClimbing(climbing, false, 0));
        // new JoystickButton(operator, 3).onTrue(new ManualClimbing(climbing, false, -0.1)).onFalse(new ManualClimbing(climbing, false, 0));
        // //climb
        // new POVButton(operator, 0).whileTrue(new ManualClimbing(climbing, true));
        // new POVButton(operator, 180).whileTrue(new ManualClimbing(climbing, false));
        // new JoystickButton(driver,6).toggleOnTrue(new ClimbingCMD(climbing, OperatorConstants.climberAngle));

        //ACTUALLY USEFUL
        new JoystickButton(driver, 2).onTrue(new ManualClimbing(climbing, false, 0.5)).onFalse(new ManualClimbing(climbing, false, 0));
        new JoystickButton(driver, 4).onTrue(new ManualClimbing(climbing, true, -0.5)).onFalse(new ManualClimbing(climbing, false, 0));

        //new JoystickButton(driver, 5).whileTrue(new AlignToReefTagRelative(drivetrain));

        new JoystickButton(driver, 5).onTrue(pathChooser.getSelected());        //new JoystickButton(driver, 5).onTrue(AutoBuilder.pathfindToPose(blueCMAlgae, constraints, 0));

        //joystick.start().onTrue(new SwerveSlowMode(0.3)).onFalse(new SwerveSlowMode(1));
        new JoystickButton(driver, 8).onTrue(new SwerveSlowMode(0.15)).onFalse(new SwerveSlowMode(1));

        //joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        new JoystickButton(driver, 1).whileTrue(drivetrain.applyRequest(() -> brake));

        new JoystickButton(driver, 10).onTrue(new ZeroIntake(intakeAngle, 59.5));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void resetDrive() {
        drivetrain.resetModules();
    }

    public void zeroIntake() {
        intakeAngle.zeroPosition();
    }

    public PathConstraints getConstraints() {
        return constraints;
    }


    // public Command getAutoDereef(String targetPoseName, Supplier<Pose2d> currentPose){


    //     if (targetPoseName.equals("FMAlgae")||targetPoseName.equals("CLAlgae")||targetPoseName.equals("CRAlgae")){
    //         return new SequentialCommandGroup(new PathfinderCMD(targetPoseName)/*, AutoBuilder.buildAuto("L2Dereef")*/);
    //     }
    //     else if (targetPoseName.equals("FRAlgae")||targetPoseName.equals("FLAlgae")||targetPoseName.equals("CMAlgae")){
    //         return new SequentialCommandGroup(new PathfinderCMD(targetPoseName)); //L3 Dereef, needs more steps
    //     }
    //     else if (targetPoseName.equals("Processor")){
    //         return new SequentialCommandGroup(new PathfinderCMD(targetPoseName) /*AutoBuilder.buildAuto("Score") */ );
    //     }
    //     else if(targetPoseName.equals("Reef")){
            
    //         Pose2d CMAlgae = PathfinderCMD.CMAlgae;
    //         Pose2d CRAlgae = PathfinderCMD.CRAlgae;
    //         Pose2d CLAlgae = PathfinderCMD.CLAlgae;
    //         Pose2d FMAlgae = PathfinderCMD.FMAlgae;
    //         Pose2d FRAlgae = PathfinderCMD.FRAlgae;
    //         Pose2d FLAlgae = PathfinderCMD.FLAlgae;

    //         Pose2d[] reefs = {CMAlgae, CRAlgae, CLAlgae, FMAlgae, FRAlgae, FLAlgae};
    //         Pose2d closestReef = CMAlgae;
    //         double minimumDistance = 1000000000;
    //         double distance;
    //         for(int i = 0; i<reefs.length; i++){
    //             distance = currentPose.get().getTranslation().getDistance(reefs[i].getTranslation());
    //             if(distance<minimumDistance){
    //                 closestReef=reefs[i];
    //                 minimumDistance=distance;
    //             }
    //             SmartDashboard.putNumber("Distance to " + i, distance);
    //         }
    //         SmartDashboard.putString("currentPose", currentPose.get().toString());
    //         return new SequentialCommandGroup(AutoBuilder.pathfindToPose(closestReef, Constants.OperatorConstants.testingConstraints, 0));  
    //     }
    //     else{
    //         return null;
    //     }
        
    // }


    public Command getAutoDereef(String targetPoseName, Supplier<Pose2d> currentPose) {
        // defer() runs the code inside the lambda ONLY when the button is pressed
        return Commands.defer(() -> {
            String finalTarget = targetPoseName;
    
            // If "Reef" is selected, find the actual name now
            if (targetPoseName.equals("Reef")) {
                finalTarget = getClosestReefName(currentPose);
            }
    
            // Return the specific command sequence for that target
            if (finalTarget.equals("FMAlgae") || finalTarget.equals("CLAlgae") || finalTarget.equals("CRAlgae")) {
                System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
                Command autoDereefL2Command;
                autoDereefL2Command = new SequentialCommandGroup(new SetIntakeAngleCMD(intakeAngle, OperatorConstants.dereefAngle), new WaitCommand(0.5), new SetIntakeAngleCMD(intakeAngle, OperatorConstants.stowAngle + 10), new WaitCommand(0.5));
                Command intakeCommand = new SequentialCommandGroup(new IntakeCMD(intaker, -0.6).raceWith(new WaitCommand(1.5)), new IntakeCMD(intaker, 0));
                return new SequentialCommandGroup( /* Commands.print("111111111111111111111111111111111111111111111111111111111111111111111111111"), new PathfinderCMD(finalTarget), Commands.print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"),*/ autoDereefL2Command.raceWith(intakeCommand) , Commands.print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"));
                
            } 
            else if (finalTarget.equals("FRAlgae") || finalTarget.equals("FLAlgae") || finalTarget.equals("CMAlgae")) {
                return new SequentialCommandGroup(new PathfinderCMD(finalTarget));
            } 
            else if (finalTarget.equals("Processor")) {
                return new SequentialCommandGroup(new PathfinderCMD(finalTarget)   , AutoBuilder.buildAuto("Score")  );
            } 
            else {
                return Commands.none();
            }
        }, Set.of(drivetrain)); // Ensures drivetrain is reserved
    }

    public String getClosestReefName(Supplier<Pose2d> currentPose) {
        Pose2d robotPose = currentPose.get();
        Pose2d[] reefs = PathfinderCMD.getAllianceReefPoses();
        String[] names = {"CMAlgae", "CRAlgae", "CLAlgae", "FMAlgae", "FRAlgae", "FLAlgae"};
    
        int closestIndex = 0;
        double minDistance = Double.MAX_VALUE;
    
        for (int i = 0; i < reefs.length; i++) {
            double dist = robotPose.getTranslation().getDistance(reefs[i].getTranslation());
            if (dist < minDistance) {
                minDistance = dist;
                closestIndex = i;
            }
        }
    
        SmartDashboard.putString("Selected Target Reef", names[closestIndex]);
        return names[closestIndex];
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)), autoChooser.getSelected());
    }

}