package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;

public class SingleLoadingCommand extends SequentialCommandGroup {
    double telescopePosition;
    double wristAngle;

    public SingleLoadingCommand(Wrist wrist, Arm arm, TargetSelection targetSelection) {
        // if targetting double substation, set telescope/wrist targets accordingly
        // otherwise set for single station

            telescopePosition = 0;
            wristAngle = Wrist.SINGLE_LOADING;
        

        // move wrist to position first, then move telescope
        addCommands(
            Commands.runOnce(() -> System.out.println(wristAngle + " " + telescopePosition)),
            new RaiseArmCommand(arm, true).withTimeout(1),
            new SetWristAngleCommand(wrist, wristAngle).withTimeout(1),
            new SetTelescopeCommand(arm, telescopePosition)
        );
    }
}
