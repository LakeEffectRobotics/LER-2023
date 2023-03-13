package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class AutoBalanceCommand extends CommandBase {
    Gyro gyro;
    Drivetrain drivetrain;
    double currentSpeed = 0.1;
    double multiplier = 0.55;
    double currentSign = 1;
    double minSpeed = 0.05;
    boolean isBackwards = false;

    public AutoBalanceCommand(Gyro g, Drivetrain d, boolean isReversed) {
        this.gyro = g;
        this.drivetrain = d;
        this.isBackwards = isReversed;
    }

    
    @Override
    public void initialize() {
        // if bot driving backwards onto charge station
        if (isBackwards) {
            currentSpeed *= -1;
            currentSign = -1;
        }
    }

    @Override
    public void execute() {
        // if bot gets within 10deg horizontal
        if ((currentSign < 0 && gyro.getPitch() > -10) 
         || (currentSign > 0 && gyro.getPitch() < 10)) {
            // change direction and slow speed by multiplier
            currentSpeed *= -multiplier;

            // make sure speed doesnt slow past minimum
            if (currentSpeed < minSpeed && currentSpeed > 0) {
                currentSpeed = minSpeed;
            } else if (currentSpeed > -minSpeed && currentSpeed < 0) {
                currentSpeed = -minSpeed;
            }

            // update current direction
            currentSign *= -1; 
        }

        // TODO: figure out proper stopping condition
        // this logic would run true everytime the robot passes 0deg? for now it works regardless
        if (Math.abs(gyro.getPitch()) < 3) {
            drivetrain.stop();
        } else {
            drivetrain.tankDrive(currentSpeed, currentSpeed);
        }
    }   

    public void end() {
        drivetrain.stop();
    }   
}
