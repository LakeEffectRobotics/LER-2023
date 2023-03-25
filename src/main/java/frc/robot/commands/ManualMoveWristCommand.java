package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class ManualMoveWristCommand extends CommandBase {
    Wrist wrist;
    DoubleSupplier incrementSupplier;
    double inc;

    public ManualMoveWristCommand(Wrist wrist, DoubleSupplier incrementSupplier) {
        addRequirements(wrist);
        this.wrist = wrist;
        this.incrementSupplier = incrementSupplier;
    }

    @Override
    public void initialize() {
    }

    
    @Override
    public void execute() {
        // axis deadzone
        if (Math.abs(incrementSupplier.getAsDouble()) < 0.2) {
            inc = 0;
        } else {
            inc = incrementSupplier.getAsDouble();
        }

        // if wrist is still alive
        if (!wrist.isWristDeadAgain) {
            // increase wrist target by joystick inc
            wrist.setTargetAngle( wrist.getTargetAngle() + inc);
        } else {
            // if wrist pot is dead, set to constant motor of the joystick cuz maybe motors are still alive
            wrist.setMotors(inc);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        if (wrist.isWristDeadAgain) {
            wrist.setMotors(0);
        }
    }
}
