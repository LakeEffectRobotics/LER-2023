package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        double Kp = -0.1f;

        double heading_error = limelight.getX();
        double steering_adjust = Kp * heading_error;

        SmartDashboard.putNumber("aim command steering adjust", steering_adjust);

        drivetrain.tankDrive(steering_adjust, -steering_adjust);
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
