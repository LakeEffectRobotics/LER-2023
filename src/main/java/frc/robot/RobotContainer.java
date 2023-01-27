// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.DriveForwardCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private Drivetrain drivetrain = new Drivetrain(RobotMap.leftController1, RobotMap.rightController1);
  public final Limelight limelight = new Limelight();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));

    configureBindings();
  }

  private void configureBindings() {
    OI.aimButton.whileTrue(new AimCommand(limelight, drivetrain));

  }

  public Command getAutonomousCommand() {
    return new DriveForwardCommand(drivetrain, 3);
  }
}
