package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;

public class SetWristAngleCommand extends InstantCommand {
    Wrist wrist;
    double angle;

    public SetWristAngleCommand(Wrist wrist, double angle) {
        this.wrist = wrist;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        wrist.setTargetAngle(angle);
    }

}
