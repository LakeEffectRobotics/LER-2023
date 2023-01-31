package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardCommand extends CommandBase {
    Drivetrain drivetrain;

    public DriveForwardCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        drivetrain.resetEncoderPosition();
    }

    @Override
    public void execute() {
        drivetrain.setSpeedOutput(drivetrain.kinematics.toWheelSpeeds(new ChassisSpeeds(1, 0, 0)));
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
