package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class OpenClawCommand extends InstantCommand {
    Claw claw;

    public OpenClawCommand(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
        claw.setPosition(Claw.Position.OPEN);
    }
}
