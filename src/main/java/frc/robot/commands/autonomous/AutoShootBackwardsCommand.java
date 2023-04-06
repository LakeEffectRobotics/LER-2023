// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SpinClawCommand;
import frc.robot.commands.SpinClawCommand.Direction;
import frc.robot.commands.instant.LowerArmCommand;
import frc.robot.commands.instant.RaiseArmCommand;
import frc.robot.commands.instant.SetWristAngleCommand;
import frc.robot.commands.instant.TransportPositionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.TargetSelection.Type;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootBackwardsCommand extends SequentialCommandGroup {
  
  // AIMING FOR MID CUBE FOR NOW
  private static final double CLAW_SPEED = 0.45;
  
  /** Creates a new AutoShootBackwardsCommand. */
  public AutoShootBackwardsCommand(Arm arm, Wrist wrist, Claw claw, TargetSelection targetSelection) {
    addCommands(
      // Raise arm and set wrist to correct angle
      new RaiseArmCommand(arm, true),
      // help wrist move to position?
      Commands.runOnce(() -> wrist.setMotors(-0.5)),

      new SetWristAngleCommand(wrist, Wrist.SCORE_CUBE_BACKWARDS),
      
      // Wait 1 second for arm + wrist to be in position
      new WaitCommand(1.3),
      
      // Spin the claw for 1 second at 100% power to shoot the cube
      new SpinClawCommand(claw, Direction.OUT, () -> CLAW_SPEED, Type.CUBE).withTimeout(0.2),
      
      // Set arm and wrist to transport position
      new TransportPositionCommand(arm, wrist)
    );
  }
}
