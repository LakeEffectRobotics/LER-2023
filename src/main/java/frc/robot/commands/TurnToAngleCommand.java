package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class TurnToAngleCommand extends CommandBase {
    Gyro gyro;
    Drivetrain drivetrain;
    double targetAngle;
    double heading_error;

    final double MIN_SPEED = 0.01;
    final double MAX_SPEED = 0.3;
    
    // could probably be tuned better but it works decently for now
    private PIDController pidController = new PIDController(0.009, 0.000001, 0.0007);

    /**
     * 
     * @param gyro
     * @param drivetrain
     * @param targetAngle absolute angle
     */
    public TurnToAngleCommand(Gyro gyro, Drivetrain drivetrain, double targetAngle) {
        this.gyro = gyro;
        this.drivetrain = drivetrain;
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {        
        heading_error = targetAngle - gyro.getAngle();
        
        double steering_adjust = pidController.calculate(heading_error);

          // min/max speed
          if (Math.abs(steering_adjust) > MAX_SPEED) {
            steering_adjust = Math.signum(steering_adjust) * MAX_SPEED;
          } else if (Math.abs(steering_adjust) < MIN_SPEED) {
            steering_adjust = Math.signum(steering_adjust) * MIN_SPEED;
          }

        drivetrain.tankDrive(steering_adjust, -steering_adjust);
    }   

    @Override
    public boolean isFinished() {
        if (Math.abs(heading_error) < 3) {
            return true;
        }
        return false;
    }

    public void end() {
        drivetrain.stop();
    }   

    public void interrupted() {
    }
}
