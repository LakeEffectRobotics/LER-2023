package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetTelescopeCommand extends CommandBase {
    Arm arm;
    double position;

    /**
     * set telescope position for the pid controller
     * @param arm
     * @param position extension distance (0, 22)
     */
    public SetTelescopeCommand(Arm arm, double position) {
        this.arm = arm;
        this.position = position;
    }

    @Override
    public void initialize() {
        arm.setTelescopePosition(position);
    }
}
