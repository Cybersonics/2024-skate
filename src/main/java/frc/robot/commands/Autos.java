// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;


import java.util.List;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.DoNothingCommand;

import frc.robot.subsystems.Drive;

import frc.robot.subsystems.NavXGyro;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command doNothing() {
    return new DoNothingCommand();
  }

  // public static Command centerRamp(Drive drive, NavXGyro gyro, Arm arm, Intake intake) {

  //   // PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Center-Ramp", 5,
  //   // 3);

  //   List<PathPlannerTrajectory> pathTrajectoryGroup = PathPlanner.loadPathGroup("Center-Ramp",
  //       new PathConstraints(1, 1), new PathConstraints(2, 1.5), new PathConstraints(2, 1.75));
  //   PPSwerveControllerCommand cubeDropDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(0), true, drive);
  //   PPSwerveControllerCommand overRampDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(1), true, drive);
  //   PPSwerveControllerCommand backToRampDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(2), true, drive);

  //   return new SequentialCommandGroup(

  //     new InstantCommand(() -> {
  //       // Reset odometry for the first path you run during auto
  //       drive.resetOdometryForState(pathTrajectoryGroup.get(0).getInitialState());
  //     }),

  //     // new InstantCommand(() -> {
  //     //   // Reset odometry for the first path you run during auto
  //     //   drive.resetOdometry(pathTrajectoryGroup.get(0).getInitialHolonomicPose());
  //     // }),

  //     cubeDropDriveCommand,
  //     overRampDriveCommand,
  //     backToRampDriveCommand,
  //     new DriveBalanceCommand(drive, gyro)
  //   );
  // }


  // private static PPSwerveControllerCommand getTrajectoryCommand(PathPlannerTrajectory pathTrajectory,
  //     boolean useAllianceColor, Drive drive) {
  //   // Define PID controllers for tracking trajectory
  //   PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  //   PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  //   PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 1, 0);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   // Return PID command
  //   return new PPSwerveControllerCommand(
  //       pathTrajectory,
  //       drive::getPose,
  //       DriveConstants.FrameConstants.kDriveKinematics,
  //       xController,
  //       yController,
  //       thetaController,
  //       drive::setModuleStates,
  //       useAllianceColor,
  //       drive);
  // }
}