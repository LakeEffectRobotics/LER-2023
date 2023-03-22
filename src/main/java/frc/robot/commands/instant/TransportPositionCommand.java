package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class TransportPositionCommand extends SequentialCommandGroup {
    Arm arm;
    Wrist wrist;

    public TransportPositionCommand(Arm arm, Wrist wrist) {
        addCommands(
            // wait until telescope position is low enough before lowering arm to avoid slam
            new SetTelescopeCommand(arm, 0)
            .until(() -> arm.getTelescopePosition() < 1),

            new LowerArmCommand(arm),

            new SetWristAngleCommand(wrist, Wrist.TRANSPORT)
        );
    }
}