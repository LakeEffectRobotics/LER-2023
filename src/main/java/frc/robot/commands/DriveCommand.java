package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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

   /* 
   @Override
    public void execute() {
        // Drive with joystick % as % of maxspeed (m/s);
        double leftInput = leftSupplier.getAsDouble(); //-0.05;
        double rightInput = rightSupplier.getAsDouble();

        if (Math.abs(leftInput - rightInput) < 0.05) {
            leftInput = rightInput;
        }

      //  final double leftSpeed = leftInput * drivetrain.MAX_SPEED * drivetrain.speedMultiplier;
      //  final double rightSpeed = rightInput * drivetrain.MAX_SPEED * drivetrain.speedMultiplier;

      //  drivetrain.velocityTankDrive(leftSpeed, rightSpeed);
      drivetrain.tankDrive(leftInput, rightInput);
    }
    */ 

    @Override
    public void execute() {
        // Drive with joystick % as % of maxspeed (m/s)
        double speed = leftSupplier.getAsDouble() * drivetrain.MAX_SPEED * drivetrain.speedMultiplier;
        // TODO make sure this works
        if (RobotContainer.demoMode.getBoolean(false)) speed = leftSupplier.getAsDouble() * drivetrain.MAX_SPEED * drivetrain.demoSpeedMultiplier;
        final double radians = Math.toRadians(360 * rightSupplier.getAsDouble());

        drivetrain.arcadeDrive(speed, radians);
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
