package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoringPositionCommand;
import frc.robot.commands.instant.SetClawCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Claw.Position;
import frc.robot.subsystems.TargetSelection.Height;
import frc.robot.subsystems.TargetSelection.Type;

public class AutoScoreConeCommand extends SequentialCommandGroup {
    public AutoScoreConeCommand(Arm arm, Wrist wrist, Claw claw) {
        addCommands(
            new ScoringPositionCommand(arm, wrist, Height.HIGH, Type.CONE),
            
            new WaitCommand(1),

            new SetClawCommand(claw, Position.OPEN)
        );
    }
}
