// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.DriveForwardCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private Drivetrain drivetrain = new Drivetrain(RobotMap.leftController1, RobotMap.rightController1);
  public final Limelight limelight = new Limelight();

  // Path planner
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New Path", new PathConstraints(0.2, 0.2));
  HashMap<String, Command> eventMap = new HashMap<>();

  // Create path planner auto builder
  RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetPose,
      new RamseteController(), drivetrain.kinematics, drivetrain::tankDrive, eventMap, drivetrain);

  // Create path command
  Command autoFollowPathCommand = autoBuilder.fullAuto(pathGroup);

  // Create robotContainer
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));

    configureBindings();
  }

  // Create button bindings
  private void configureBindings() {
    OI.aimButton.whileTrue(new AimCommand(limelight, drivetrain));

  }

  // Set autonomous command
  public Command getAutonomousCommand() {
    return autoFollowPathCommand;
  }
}
