// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CurtisDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain drivetrain = new Drivetrain(RobotMap.leftController1, RobotMap.rightController1);

  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));
    
    configureBindings();
  }

  private void configureBindings() {
    OI.curtisStraightButton.whileTrue(new CurtisDriveCommand(drivetrain));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
