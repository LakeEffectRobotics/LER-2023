package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ApriltagAimCommand extends CommandBase {
    Limelight limelight;
    Drivetrain drivetrain;

    private PIDController pidController = new PIDController(0.01, 0.0, 0.0);

    public ApriltagAimCommand(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        limelight.getPose();

        double heading_error = limelight.getX();
        double steering_adjust = pidController.calculate(heading_error);
        drivetrain.tankDrive(-steering_adjust, steering_adjust);
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
