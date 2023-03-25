package frc.robot.commands.instant;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;

public class DoubleLoadingCommand extends SequentialCommandGroup {
    Wrist wrist;
    Arm arm;
    TargetSelection targetSelection;
    BooleanSupplier useDoublestationSupplier; 

    double telescopePosition;
    double wristAngle;

    public DoubleLoadingCommand(Wrist wrist, Arm arm, TargetSelection targetSelection) {
        // if targetting double substation, set telescope/wrist targets accordingly
        // otherwise set for single station
            telescopePosition = Arm.DOUBLE_LOADING;
            wristAngle = Wrist.DOUBLE_LOADING;


        // move wrist to position first, then move telescope
        addCommands(
            new SetWristAngleCommand(wrist, wristAngle),
            // if wrist is dead, continue; otherwise, wait till wrist is in right spot
            // if wrist is being stupid and doesnt get within 10deg, just go anyway atfter 1 sec
            new ConditionalCommand(new WaitCommand(0.1), new WaitUntilCommand(() -> Math.abs(wrist.getCurrentAngle() - wristAngle) < 10), () -> wrist.isWristDeadAgain)
            .withTimeout(1),

            new RaiseArmCommand(arm, true),
            new WaitCommand(0.5),

            new SetTelescopeCommand(arm, telescopePosition)
        );
    }
}
