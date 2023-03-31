package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.TargetSelection.Height;

/**
 * spin claw out for time and speed depending on selected target
 */
public class ShootScoreCommand extends CommandBase {
    Claw claw;
    Height height;
    TargetSelection targetSelection;
    boolean auto = false;

    double time = 700;
    double speed;
    double startTime;

    /**
     * spin claw power based on target selection height
     * @param targetSelection
     * @param claw
     */
    public ShootScoreCommand(TargetSelection targetSelection, Claw claw) {
        this.claw = claw;
        this.targetSelection = targetSelection;
        auto = false;
    }

    /**
     * shoot score given a specific node, for use in autonomous
     * @param height
     * @param claw
     */
    public ShootScoreCommand(Height height, Claw claw) {
        this.height = height;
        this.claw = claw;
        auto = true;
    }

    @Override
    public void initialize() {
        // if height wasnt initialized (used during auto), use LIVE target selection
        if (!auto) {
            this.height = targetSelection.getSelectedNode().getHeight();
        }

        // set for score mid/high cube forwards
        if (height == Height.HIGH) {
            speed = 0.75;
        } else if (height == Height.MID) {
            speed = 0.50;
        } else if (height == Height.LOW) {
            // set for scoring low cube BACKWARDS
            speed = 0.35;
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
