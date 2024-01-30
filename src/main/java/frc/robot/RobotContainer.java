// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;

import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static NavXGyro _gyro = NavXGyro.getInstance(); // This must be called before Drive as it is used by the Drive
  public static Drive _drive = Drive.getInstance(_gyro);
  
  // public final CommandJoystick leftStick = new CommandJoystick(OperatorConstants.LeftStick);
  // public final CommandJoystick rightStick = new CommandJoystick(OperatorConstants.RightStick);

  public final CommandXboxController xboxController = new CommandXboxController(2);

  // Setup Sendable chooser for picking autonomous program in SmartDashboard
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));


    CommandScheduler.getInstance()
      //.setDefaultCommand(_drive, new DriveCommand(_drive, driveController, _gyro));
      // .setDefaultCommand(_drive, new DriveCommand(_drive, leftStick, rightStick, _gyro));      
      .setDefaultCommand(_drive, new DriveCommand(_drive, xboxController, _gyro));



    // Configure Autonomous Options
    autonomousOptions();

    // Configure the trigger bindings
    configureBindings(); 

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset NavX
    // leftStick.button(7).onTrue(new InstantCommand(() -> _gyro.zeroNavHeading(), _gyro));

    //  rightStick.button(3).toggleOnTrue(new ConditionalCommand(
    //      new InstantCommand(() -> _drive.setDriveModeBrake()),
    //      new InstantCommand(() -> _drive.setDriveModeCoast()),
    //      () -> _drive.toggleMode()
    //    )
    //   );

      xboxController.povDown().onTrue(new InstantCommand(() -> _gyro.zeroNavHeading(), _gyro));

      xboxController.x().toggleOnTrue(new ConditionalCommand(
         new InstantCommand(() -> _drive.setDriveModeBrake()),
         new InstantCommand(() -> _drive.setDriveModeCoast()),
         () -> _drive.toggleMode()
       )
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get the selected Auto in smartDashboard
    return m_chooser.getSelected();
  }

  /**
   * Use this to set Autonomous options for selection in Smart Dashboard
   */
  private void autonomousOptions() {
    // Example adding Autonomous option to chooser

    m_chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", m_chooser);

    //m_chooser.addOption("Do Nothing", Autos.doNothing());
    // m_chooser.addOption("Cable Straight", Autos.cableDriveStraight(_drive, _arm, _intake));
    // m_chooser.addOption("Center Ramp", Autos.centerRamp(_drive, _gyro, _arm, _intake));
    // m_chooser.addOption("Center Ramp Cube", Autos.centerRampCube(_drive, _gyro, _arm, _intake));
    // //m_chooser.addOption("Barrier Straight", Autos.barrierDriveStraight(_drive));
    // m_chooser.addOption("Far Barrier Cube Score", Autos.farBarrierCubeScoreLow(_drive, _intake, _arm));
    // m_chooser.addOption("Barrier Cone", Autos.barrierCone(_drive, _gyro, _intake, _arm));
    // m_chooser.addOption("Blue Barrier Cone Ramp", Autos.blueBarrierConeRamp(_drive, _gyro, _intake, _arm));
    // m_chooser.addOption("DO NOT RUN - Jeremy Only", Autos.centerRampCubeTest(_drive, _gyro, _arm, _intake));

    
    // Put the chooser on the dashboard
    //SmartDashboard.putData(m_chooser);
  }
}
