// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ApriltagAimCommand;
import frc.robot.commands.ApriltagPoseCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CurtisDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SpinClawCommand;
import frc.robot.commands.SpinClawCommand.Direction;
import frc.robot.commands.instant.SetClawCommand;
import frc.robot.subsystems.Arm;
import frc.robot.commands.instant.SetClawCommand;
import frc.robot.commands.instant.SetClawCommand;
import frc.robot.subsystems.Claw;
import frc.robot.commands.instant.SetWristAngleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Claw.Position;

public class RobotContainer {

  private Drivetrain drivetrain = new Drivetrain(RobotMap.leftController1, RobotMap.rightController1);
  public final Limelight limelight = new Limelight();
  public final Wrist wrist = new Wrist(RobotMap.wristController);
  private Gyro gyro = new Gyro();
  public final Arm arm = new Arm(RobotMap.telescopeController1, RobotMap.telescopeController2, RobotMap.leftArmSolenoid, RobotMap.rightArmSolenoid);
  private Claw claw = new Claw(RobotMap.leftClawController, RobotMap.rightClawController, RobotMap.leftClawSolenoid, RobotMap.rightClawSolenoid);

  // Dashboard autonomous chooser
  public final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Path planner
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("drivearound", new PathConstraints(2, 1));
  HashMap<String, Command> eventMap = new HashMap<>();

  // Create path planner auto builder
  RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetPose,
      new RamseteController(), drivetrain.kinematics, drivetrain::velocityTankDrive, eventMap, drivetrain);

  // Create path command
  Command autoFollowPathCommand = autoBuilder.fullAuto(pathGroup);

  // Create robotContainer
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));
    gyro.setDefaultCommand(new GyroCommand(gyro));

    // Put autonomous chooser on dashboard
    autoChooser.addOption("new path", autoFollowPathCommand);
    autoChooser.addOption("arm angle", new SetWristAngleCommand(wrist, 0));

    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  // Create button bindings
  private void configureBindings() {
    OI.aimButton.whileTrue(new ApriltagAimCommand(limelight, drivetrain));
    OI.resetPoseButton.whileTrue(new ApriltagPoseCommand(limelight, drivetrain));
    
    OI.curtisStraightButton.whileTrue(new CurtisDriveCommand(drivetrain));

    OI.openClawButton.onTrue(new SetClawCommand(claw, Position.OPEN));
    OI.closeClawButton.onTrue(new SetClawCommand(claw, Position.CLOSED));
    OI.spinInButton.whileTrue(new SpinClawCommand(claw, Direction.IN, OI.clawInSpeedSupplier));
    OI.spitOutButton.whileTrue(new SpinClawCommand(claw, Direction.OUT, OI.clawOutSpeedSupplier));
  }

  // Set autonomous command from dashboard choice
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
