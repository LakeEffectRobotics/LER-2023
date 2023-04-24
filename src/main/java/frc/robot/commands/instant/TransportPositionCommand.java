package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class TransportPositionCommand extends SequentialCommandGroup {
    Arm arm;
    Wrist wrist;

    public TransportPositionCommand(Arm arm, Wrist wrist) {
        addCommands(
            new ConditionalCommand(
                // if arm is dead, do nothing
                new WaitCommand(0.01),
                // if arm is alive, wait until arm is down before lowering pistons in order to avoid slamming the entire extended arm on the floor which is always highly unpleasant!
                new SetTelescopeCommand(arm, 0)
                .until(() -> arm.getTelescopePosition() < 1),
                // check for arm deadness
                () -> arm.isArmDead),

            new LowerArmCommand(arm),

            new SetWristAngleCommand(wrist, Wrist.TRANSPORT)
        );
    }
}