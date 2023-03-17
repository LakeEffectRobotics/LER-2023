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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootBackwardsCommand extends SequentialCommandGroup {
  
  private static final double CLAW_SPEED = 0.73;
  
  /** Creates a new AutoShootBackwardsCommand. */
  public AutoShootBackwardsCommand(Arm arm, Wrist wrist, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Raise arm and set wrist to correct angle
      new RaiseArmCommand(arm),
      new SetWristAngleCommand(wrist, Wrist.SCORE_CUBE_BACKWARDS),
      
      // Wait 1 second for arm + wrist to be in position
      new WaitCommand(1),
      
      // Spin the claw for 1 second at 100% power to shoot the cube
      new SpinClawCommand(claw, Direction.OUT, () -> CLAW_SPEED).withTimeout(0.3),
      
      // Set arm and wrist to transport position
      new SetWristAngleCommand(wrist, Wrist.TRANSPORT),
      new LowerArmCommand(arm)
    );
  }
}
