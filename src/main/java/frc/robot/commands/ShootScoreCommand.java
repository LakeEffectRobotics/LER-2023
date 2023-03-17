package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.TargetSelection.Height;

/**
 * spin claw out for time and speed depending on selected target
 */
public class ShootScoreCommand extends CommandBase {
    TargetSelection targetSelection;
    Claw claw;

    
    double time = 700;
    double speed;
    double startTime;
    Height height;

    public ShootScoreCommand(TargetSelection targetSelection, Claw claw) {
        this.targetSelection = targetSelection;
        this.claw = claw;
    }

    @Override
    public void initialize() {
        this.height = targetSelection.getSelectedNode().getHeight();
        // set for score mid/high cube forwards
        if (height == Height.HIGH) {
            speed = 0.85;
        } else if (height == Height.MID) {
            speed = 0.5;
        } else if (height == Height.LOW) {
            // set for scoring low cube BACKWARDS
            speed = 0.4;
        }

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        claw.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        if (time != 0) {
            if (System.currentTimeMillis() - startTime >= time) {
                return true;
            }
            return false;
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
