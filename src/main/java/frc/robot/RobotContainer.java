// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ApriltagAimCommand;
import frc.robot.commands.ApriltagPoseCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.CurtisDriveCommand;
import frc.robot.commands.DiscoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.pathplannerUtils.CreatePathUtils;
import frc.robot.commands.SpinClawCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.SpinClawCommand.Direction;
import frc.robot.commands.autonomous.AutoIntakeCommand;
import frc.robot.commands.autonomous.AutoShootBackwardsCommand;
import frc.robot.commands.instant.SetClawCommand;
import frc.robot.commands.instant.LowerArmCommand;
import frc.robot.commands.instant.RaiseArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.commands.instant.SetWristAngleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Lights.Colour;
import frc.robot.subsystems.TargetSelection.Height;
import frc.robot.commands.GyroCommand;
import frc.robot.commands.ManualMoveWristCommand;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Claw.Position;
import frc.robot.subsystems.Lights;

public class RobotContainer {

  // Initialize subsystems
  public Drivetrain drivetrain = new Drivetrain(RobotMap.leftController1, RobotMap.rightController1);
  public final Limelight limelight = new Limelight();
  public Gyro gyro = new Gyro();
  public final Arm arm = new Arm(RobotMap.telescopeController1, RobotMap.telescopeController2, RobotMap.leftArmSolenoid, RobotMap.rightArmSolenoid);
  public Claw claw = new Claw(RobotMap.rightClawController, RobotMap.leftClawSolenoid, RobotMap.rightClawSolenoid);
  public static final Lights lights = new Lights();
  public final Wrist wrist = new Wrist(RobotMap.wristController, arm);

  // path utils
  CreatePathUtils createPathUtils = new CreatePathUtils(drivetrain, limelight, arm, wrist, claw, gyro);
  
  private TargetSelection targetSelection = new TargetSelection();

  // Dashboard autonomous chooser
  public final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Create robotContainer
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));
    gyro.setDefaultCommand(new GyroCommand(gyro));
    wrist.setDefaultCommand(new ManualMoveWristCommand(wrist, OI.manualMoveWristSupplier));

    // Put autonomous chooser on dashboard
    autoChooser.addOption("arm angle", new SetWristAngleCommand(wrist, 0));
    
    autoChooser.addOption("flat 2 cube", createPathUtils.createPathCommand("flat 2 cube", 2.5, 1));
    autoChooser.addOption("bump 2 cube turn", createPathUtils.createPathCommand("bump 2 cube turn", 1, 1));
    autoChooser.addOption("balance 1 cube", createPathUtils.createPathCommand("balance 1 cube", 2.5, 1));
    autoChooser.addOption("balance 2 cube", createPathUtils.createPathCommand("balance 2 cube", 2.5, 1));

    autoChooser.addOption("outtake", new AutoShootBackwardsCommand(arm, wrist, claw));
    autoChooser.addOption("intake", new AutoIntakeCommand(drivetrain, arm, wrist, claw));

    SmartDashboard.putData(autoChooser);
    configureBindings();

    lights.setBoth(Colour.PURPLE);

    CameraServer.startAutomaticCapture();
  }

  // Create button bindings
  private void configureBindings() {
    OI.aimButton.whileTrue(new ApriltagAimCommand(limelight, drivetrain));
    OI.resetPoseButton.whileTrue(new ApriltagPoseCommand(limelight, drivetrain));
    
    OI.turnButton.whileTrue(new TurnToAngleCommand(gyro, drivetrain, 0));
    OI.curtisStraightButton.whileTrue(new CurtisDriveCommand(drivetrain));

    OI.openClawButton.onTrue(new SetClawCommand(claw, Position.OPEN));
    OI.closeClawButton.onTrue(new SetClawCommand(claw, Position.CLOSED));
    OI.spinInButton.whileTrue(new SpinClawCommand(claw, Direction.IN, OI.clawInSpeedSupplier));
    OI.spitOutButton.whileTrue(new SpinClawCommand(claw, Direction.OUT, OI.clawOutSpeedSupplier));
    
    // Move arm and wrist into transport position
    OI.transportButton.onTrue(new LowerArmCommand(arm).andThen(new SetWristAngleCommand(wrist, Wrist.TRANSPORT)));

    // Loading station position
    OI.loadingStationButton.onTrue(new RaiseArmCommand(arm).andThen(new SetWristAngleCommand(wrist, Wrist.LOADING_STATION)));

    // Move arm and wrist into ground intake position. Only run if arm is already down to avoid smashing things in front of the robot. (Still sets arm to down position for completion sake)
    OI.groundIntakeButton.onTrue(
      new ConditionalCommand(
        new LowerArmCommand(arm).andThen(new SetWristAngleCommand(wrist, Wrist.GROUND)),
        Commands.runOnce(() -> System.out.print("Lower arm before going to ground position")), 
        () -> arm.getArmPosition() == ArmPosition.DOWN
      )
    );
    
    // Temp scoring position for early testing
    // OI.scorePositionButton.onTrue(new RaiseArmCommand(arm).andThen(new SetWristAngleCommand(wrist, Wrist.SCORE_CONE)));

    OI.dicoButton.whileTrue(new DiscoCommand(lights));
  }

  // Set autonomous command from dashboard choice
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
