package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SpitOutCommand extends CommandBase {
    Claw claw;

    public SpitOutCommand(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setSpeed(1);
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
