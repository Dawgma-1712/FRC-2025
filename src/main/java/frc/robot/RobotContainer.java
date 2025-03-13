// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// auto imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.intake.IntakeAngleCMD;
import frc.robot.commands.intake.IntakeCMD;
import frc.robot.commands.crossbow.ManualCrossbowCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.Constants.OperatorConstants;
import frc.robot.commands.crossbow.*;

public class RobotContainer {
    public static double speed = 1;

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

    public final CommandSwerveDrivetrain drivetrain;
    
    // subsystems
    public final Intaker intaker = new Intaker();
    public final IntakeAngle intakeAngle = new IntakeAngle();
    public final Crossbow crossbow = new Crossbow();
    public final Climbing climbing = new Climbing();

    private final Joystick driver = new Joystick(0); 
    private final Joystick operator = new Joystick(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();

        Command autoDereefL2Command;
        autoDereefL2Command = new SequentialCommandGroup(new SetIntakeAngleCMD(intakeAngle, OperatorConstants.dereefAngle), new WaitCommand(0.5), new SetIntakeAngleCMD(intakeAngle, OperatorConstants.stowAngle), new WaitCommand(0.5), new IntakeCMD(intaker, 0));
        Command intakeCommand = new SequentialCommandGroup(new IntakeCMD(intaker, -0.6), new WaitCommand(3));
        NamedCommands.registerCommand("L2Dereef", autoDereefL2Command.raceWith(intakeCommand));

        Command autoScoreCommand;
        autoScoreCommand = new SequentialCommandGroup(new SetIntakeAngleCMD(intakeAngle, OperatorConstants.stowAngle), new WaitCommand(1), new IntakeCMD(intaker, 0));
        NamedCommands.registerCommand("Score", autoScoreCommand.raceWith(new IntakeCMD(intaker, -0.6)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
         4).toggleOnTrue(new IntakeAngleCMD(intakeAngle));
        new JoystickButton(driver, 3).toggleOnTrue(new CrossbowCMD(crossbow, true));
        new JoystickButton(driver, 2).toggleOnTrue(new CrossbowCMD(crossbow, false));

        // new JoystickButton(operator, 2).onTrue(new ManualClimbing(climbing, false, 0.1)).onFalse(new ManualClimbing(climbing, false, 0));
        // new JoystickButton(operator, 3).onTrue(new ManualClimbing(climbing, false, -0.1)).onFalse(new ManualClimbing(climbing, false, 0));
        // //climb
        // new POVButton(operator, 0).whileTrue(new ManualClimbing(climbing, true));
        // new POVButton(operator, 180).whileTrue(new ManualClimbing(climbing, false));
        // new JoystickButton(driver,6).toggleOnTrue(new ClimbingCMD(climbing, OperatorConstants.climberAngle));

        //ACTUALLY USEFUL
        new JoystickButton(operator, 2).onTrue(new ManualClimbing(climbing, false, 0.5)).onFalse(new ManualClimbing(climbing, false, 0));
        new JoystickButton(operator, 3).onTrue(new ManualClimbing(climbing, true, -0.5)).onFalse(new ManualClimbing(climbing, false, 0));

        //joystick.start().onTrue(new SwerveSlowMode(0.3)).onFalse(new SwerveSlowMode(1));
        new JoystickButton(driver, 8).onTrue(new SwerveSlowMode(0.3)).onFalse(new SwerveSlowMode(1));

        //joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        new JoystickButton(driver, 1).whileTrue(drivetrain.applyRequest(() -> brake));
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

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}