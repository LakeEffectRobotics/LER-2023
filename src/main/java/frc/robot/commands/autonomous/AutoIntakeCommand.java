// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SpinClawCommand;
import frc.robot.commands.SpinClawCommand.Direction;
import frc.robot.commands.instant.LowerArmCommand;
import frc.robot.commands.instant.SetWristAngleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeCommand extends SequentialCommandGroup {
  
  // Note that the claw subsystem applies it's own scaling, so 100% should be the optimal speed
  private static final double CLAW_SPEED = 1;

  private static final double TIMEOUT = 5;

  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(Drivetrain drivetrain, Arm arm, Wrist wrist, Claw claw, TargetSelection targetSelection) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Lower the wrist and arm, 
      new LowerArmCommand(arm),
      new SetWristAngleCommand(wrist, Wrist.GROUND),
      new WaitCommand(0.5),

      // Spin claw and drive forward until limit switch is pressed
      new ParallelCommandGroup(
       // new DriveCommand(drivetrain, () -> 0.25, () -> 0.25)
        new SpinClawCommand(claw, Direction.IN, () -> CLAW_SPEED, targetSelection)//, 
      ).until(() -> claw.GetLimitPressed()).withTimeout(TIMEOUT),

      // Raise wrist to transport position
      new SetWristAngleCommand(wrist, Wrist.TRANSPORT)
    );
  }
}
