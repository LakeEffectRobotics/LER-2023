package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class RaiseArmCommand extends InstantCommand {
    Arm arm;

    public RaiseArmCommand(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.raiseArm();
    }
}
