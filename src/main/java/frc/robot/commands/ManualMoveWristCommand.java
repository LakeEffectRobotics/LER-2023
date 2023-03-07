package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class ManualMoveWristCommand extends CommandBase {
    Wrist wrist;
    DoubleSupplier incrementSupplier;

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
        double currentAngle = wrist.getCurrentAngle();
        
        // increase wrist target by joystick
        wrist.setTargetAngle( currentAngle + incrementSupplier.getAsDouble());
    }
}
