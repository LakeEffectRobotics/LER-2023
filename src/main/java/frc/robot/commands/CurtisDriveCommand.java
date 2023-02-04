package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class CurtisDriveCommand extends CommandBase {
    
    Drivetrain drivetrain;

    public CurtisDriveCommand(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.stop();
    }

    @Override
    public void execute() {
        if (OI.curtisLeftButton.getAsBoolean()) {
            drivetrain.setOutput(0.15, 0.2);
        } else if (OI.curtisRightButton.getAsBoolean()) {
            drivetrain.setOutput(0.2, 0.15);
        } else {
            drivetrain.setOutput(0.2, 0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
	public boolean isFinished() {
		return false;
	}
}