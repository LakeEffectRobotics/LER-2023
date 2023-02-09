package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FollowPathCommand extends CommandBase {
    Drivetrain drivetrain;
    double voltage;

    public FollowPathCommand(Drivetrain drivetrain, double voltage) {
        this.drivetrain = drivetrain;
        this.voltage = voltage;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        drivetrain.resetEncoderPosition();
    }

    @Override
    public void execute() {
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
