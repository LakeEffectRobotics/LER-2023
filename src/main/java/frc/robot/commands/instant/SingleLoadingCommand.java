package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;

public class SingleLoadingCommand extends SequentialCommandGroup {
    double telescopePosition;
    double wristAngle;

    public SingleLoadingCommand(Wrist wrist, Arm arm, TargetSelection targetSelection) {
            telescopePosition = 0;
            wristAngle = Wrist.SINGLE_LOADING;
        
        // drop tellescope first, then set wrist because telescope is going to 0 and its better to move wrist there than if its still up
        addCommands(
            new RaiseArmCommand(arm, true),

            new ConditionalCommand(
                //if wrist very back, give kick
                Commands.runOnce(() -> wrist.setMotors(-0.7)),
                //otherwise, no help needed
                new WaitCommand(telescopePosition),
                
                () -> wrist.getCurrentAngle() > 120),

            new SetTelescopeCommand(arm, telescopePosition).withTimeout(0.3),

            new SetWristAngleCommand(wrist, wristAngle)
        );
    }
}
