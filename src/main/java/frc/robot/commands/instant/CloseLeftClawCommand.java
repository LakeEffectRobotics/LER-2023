package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseLeftClawCommand extends CommandBase {
    Claw claw;

    public CloseLeftClawCommand(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
        claw.setPosition(Claw.Position.CLOSED);
    }
}
