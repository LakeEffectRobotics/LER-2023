package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class TimedSpinClawCommand extends CommandBase {
    Claw claw;
    double time = 0;
    double speed;
    double startTime;

    /**
     * spin claw for a time
     * @param claw
     * @param speed +ve spins out, -ve spins in (-1,1)
     * @param time ms
     */
    public TimedSpinClawCommand(Claw claw, double speed, double time){
        this.claw = claw;
        this.speed = speed;
        this.time = time;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

        @Override
    public void execute() {
        claw.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startTime >= time) {
            return true;
        }
        
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
