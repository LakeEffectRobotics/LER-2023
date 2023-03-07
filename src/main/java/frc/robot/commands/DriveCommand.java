package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {

    Drivetrain drivetrain;

    DoubleSupplier leftSupplier;
    DoubleSupplier rightSupplier;

    public DriveCommand(Drivetrain drivetrain, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;
    }

    @Override
    public void initialize() {
        drivetrain.stop();
    }

    @Override
    public void execute() {
        // Drive with joystick % as % of maxspeed (m/s)
        final double leftSpeed = leftSupplier.getAsDouble() * drivetrain.MAX_SPEED;
        final double rightSpeed = rightSupplier.getAsDouble() * drivetrain.MAX_SPEED;
        SmartDashboard.putNumber("leftspeed target", leftSpeed);
        SmartDashboard.putNumber("rightspeed target", rightSpeed);
        drivetrain.velocityTankDrive(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("joystik left", leftSupplier.getAsDouble());

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
