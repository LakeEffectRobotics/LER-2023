package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.TargetSelection.Height;
import frc.robot.subsystems.TargetSelection.Node;

/**
 * spin claw out for time and speed depending on selected target
 */
public class ShootScoreCommand extends CommandBase {
    Claw claw;

    
    double time = 700;
    double speed;
    double startTime;
    Height height;
    Node node;
    TargetSelection targetSelection;

    /**
     * shoot score 
     * @param targetSelection
     * @param claw
     */
    public ShootScoreCommand(TargetSelection targetSelection, Claw claw) {
        this.claw = claw;
        this.targetSelection = targetSelection;
    }

    /**
     * shoot score given a specific node, for use in autonomous
     * @param node
     * @param claw
     */
    public ShootScoreCommand(Node node, Claw claw) {
        this.node = node;
        this.claw = claw;
    }

    @Override
    public void initialize() {
        // if initalized in auto with a node, height is node height
        if (node != null) {
            this.height = node.getHeight();
        } else {
            // otherwise, get CURRENT LIVE targetselection height
            this.height = targetSelection.getSelectedNode().getHeight();
        }

        // set for score mid/high cube forwards
        if (height == Height.HIGH) {
            speed = 0.85;
        } else if (height == Height.MID) {
            speed = 0.55;
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
