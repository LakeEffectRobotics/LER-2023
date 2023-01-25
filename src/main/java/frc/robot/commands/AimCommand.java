package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AimCommand extends CommandBase {
    Limelight limelight;
    Drivetrain drivetrain;

    public AimCommand(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double Kp = 0.06f;
        double heading_error = limelight.getX();
        double steering_adjust = Kp * heading_error;
        double min_command = 0.6f;

        if (limelight.getX() > 1.0) {
            steering_adjust = Kp * heading_error - min_command;
        } else if (limelight.getX() < 1.0) {
            steering_adjust = Kp * heading_error + min_command;
        }

        drivetrain.setOutput(steering_adjust, -steering_adjust);
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
