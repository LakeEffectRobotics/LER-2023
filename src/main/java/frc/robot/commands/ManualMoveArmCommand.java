package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualMoveArmCommand extends CommandBase {
    Arm arm;
    DoubleSupplier incrementSupplier;
    double inc;

    public ManualMoveArmCommand(Arm arm, DoubleSupplier incrementSupplier) {
        this.arm = arm;
        this.incrementSupplier = incrementSupplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // axis deadzone
        if (Math.abs(incrementSupplier.getAsDouble()) < 0.2) {
            inc = 0;
        } else {
            inc = incrementSupplier.getAsDouble();
        }
        arm.setTelescopePosition(arm.telescopeTargetPosition + inc);
    }
}
