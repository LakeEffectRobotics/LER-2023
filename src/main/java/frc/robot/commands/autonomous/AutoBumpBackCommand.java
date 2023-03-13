package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * simply moves bot backwards a few inches at the end of auto so its close to high node
 */
public class AutoBumpBackCommand extends CommandBase {
    double time;
    double startTime;
    Drivetrain drivetrain;

    /**
     * 
     * @param time milliseconds
     * @param drivetrain
     */
    public AutoBumpBackCommand(double time, Drivetrain drivetrain) {
        this.time = time;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        drivetrain.tankDrive(-0.1, -0.1);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startTime > time) {
            return true;
        }
        return false;
    }

    public void end() {
        drivetrain.stop();
    }  
    
}
