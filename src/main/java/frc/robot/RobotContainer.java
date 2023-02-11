// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ApriltagAimCommand;
import frc.robot.commands.ApriltagPoseCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CurtisDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.pathplannerUtils.CreatePathUtils;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private Drivetrain drivetrain = new Drivetrain(RobotMap.leftController1, RobotMap.rightController1);
  public final Limelight limelight = new Limelight();

  // path utils
  CreatePathUtils createPathUtils = new CreatePathUtils(drivetrain, limelight);

  // Dashboard autonomous chooser
  public final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Create robotContainer
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));

    // Put autonomous chooser on dashboard
    autoChooser.addOption("new path", createPathUtils.createPathCommand("drivearound", 2, 2));

    SmartDashboard.putData(autoChooser);
    configureBindings();

    // Start pathplanner visualizer server
    // DISABLE FOR COMPETITION to save on network
    PathPlannerServer.startServer(5811);
  }

  // Create button bindings
  private void configureBindings() {
    OI.aimButton.onTrue(
        createPathUtils.createOntheflyPath(drivetrain::getPose,
            new Pose2d(new Translation2d(15.3, 6.6), new Rotation2d()), 1, 1));
    OI.resetPoseButton.whileTrue(new ApriltagPoseCommand(limelight, drivetrain));
    // OI.curtisStraightButton.whileTrue(new CurtisDriveCommand(drivetrain));
  }

  // Set autonomous command from dashboard choice
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
