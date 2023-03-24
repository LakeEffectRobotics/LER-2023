package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class RaiseArmCommand extends InstantCommand {
    Arm arm;
    boolean raiseBoth = false;

    public RaiseArmCommand(Arm arm, boolean raiseBoth) {
        this.arm = arm;
        this.raiseBoth = raiseBoth;
    }

    @Override
    public void initialize() {
        if (raiseBoth) {
            arm.raiseBothPistons();
        } else {
            arm.raiseOnePiston();
        }
    }
}
