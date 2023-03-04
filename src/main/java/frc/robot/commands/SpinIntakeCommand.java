package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SpinIntakeCommand extends CommandBase {
    
    Claw claw;
    DoubleSupplier speedSupplier;

    /**
     * Create a new SpinIntakeCommand
     * @param claw Claw subsystem
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins in
     */
    public SpinIntakeCommand(Claw claw, DoubleSupplier speedSupplier) {
        addRequirements(claw);
        this.claw = claw;
        this.speedSupplier = speedSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setSpeed(-speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
