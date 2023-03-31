package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class AutoBalanceCommand extends CommandBase {
    Gyro gyro;
    Drivetrain drivetrain;
    PIDController pidController = new PIDController(0.005, 0, 0);
    double currentSpeed;

    public AutoBalanceCommand(Gyro gyro, Drivetrain drivetrain) {
        this.gyro = gyro;
        this.drivetrain = drivetrain;
    }

    
    @Override
    public void initialize() {
        pidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        // if we are level, stop moving!!!
        // A CHARGE STATION is considered LEVEL if it is within approximately 2½° of parallel to FIELD carpet
        if (Math.abs(gyro.getPitch()) < 2.5) {
            drivetrain.stop();
        } else {
            // otherwise, pid towards 0 deg
            currentSpeed = pidController.calculate(gyro.getPitch());

            drivetrain.tankDrive(-currentSpeed, -currentSpeed);    
        }
    }   

    // TODO: proper end condition, or not because auto times out anyway and we never do anything after balancing
    public boolean isFinished() {
        return false;
    }

    public void end() {
        drivetrain.stop();
    }   
}
