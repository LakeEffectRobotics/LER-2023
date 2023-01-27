package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardCommand extends CommandBase {
    Drivetrain drivetrain;
    int target;

    public DriveForwardCommand(Drivetrain drivetrain, int target) {
        this.drivetrain = drivetrain;
        this.target = target;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        drivetrain.resetEncoderPosition();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("test auto", "hi");
        drivetrain.setOutput(0.2, 0.2);
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
