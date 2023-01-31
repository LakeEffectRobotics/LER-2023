package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FunScienceExpCommand extends CommandBase {
    Drivetrain drivetrain;
    double volts;

    public FunScienceExpCommand(Drivetrain drivetrain, double volts) {
        this.volts = volts;
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
        drivetrain.leftLeadController.setVoltage(volts);
        drivetrain.rightLeadController.setVoltage(volts);

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
