package frc.robot.commands.instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.TargetSelection.Height;
import frc.robot.subsystems.TargetSelection.Type;

public class ScoringPositionCommand  extends InstantCommand {
    Arm arm;
    Wrist wrist;
    TargetSelection targetSelection;

    public ScoringPositionCommand(Arm arm, Wrist wrist, TargetSelection targetSelection){
        this.arm = arm;
        this.wrist = wrist;
        this.targetSelection = targetSelection;
    }

    @Override
    public void initialize() {
        if (targetSelection.getSelectedNode().getType() == Type.CONE) {
            // move wrist for cone
            wrist.setTargetAngle(Wrist.SCORE_CONE);

            // extend telescope based on target selection height
            // raise one or both pistons according to height as mid doesnt need 2
            if (targetSelection.getSelectedNode().getHeight() == Height.HIGH) {
                arm.raiseBothPistons();
                arm.setTelescopePosition(Arm.HIGH_CONE);
            } else if (targetSelection.getSelectedNode().getHeight() == Height.MID) {
                arm.raiseBothPistons();
                arm.setTelescopePosition(Arm.MID_CONE);
            }
        } else if (targetSelection.getSelectedNode().getType() == Type.CUBE) {
            // lower arm for cubes in case its raised for some reason
            arm.lowerArm();

            // move wrist to scoring position accordingly
            if (targetSelection.getSelectedNode().getHeight() == Height.HIGH) {
                wrist.setTargetAngle(Wrist.SCORE_CUBE_FORWARD);
            } else if (targetSelection.getSelectedNode().getHeight() == Height.MID) {
                wrist.setTargetAngle(Wrist.SCORE_CUBE_FORWARD);
            } else if (targetSelection.getSelectedNode().getHeight() == Height.LOW) {
                // transport position is conveniently also good for low cube backwards
                wrist.setTargetAngle(Wrist.TRANSPORT);
            } 
        }
    }
}
