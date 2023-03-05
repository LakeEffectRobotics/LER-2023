package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.Position;

public class SetClawCommand extends InstantCommand {
    
    Claw claw;
    Position position;

    public SetClawCommand(Claw claw, Position position) {
        // Don't add claw as requirement, as this shouldn't interrupt spin command
        this.claw = claw;
        this.position = position;
    }
    
    @Override
    public void initialize() {
        claw.setPosition(position);
    }
}
