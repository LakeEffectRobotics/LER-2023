package frc.robot.commands.instant;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.TargetSelection.Height;
import frc.robot.subsystems.TargetSelection.Node;
import frc.robot.subsystems.TargetSelection.Type;

public class ScoringPositionCommand  extends InstantCommand {
    Arm arm;
    Wrist wrist;
    TargetSelection targetSelection;
    Supplier<Node> targetSupplier;

    public ScoringPositionCommand(Arm arm, Wrist wrist, TargetSelection targetSelection, Supplier<Node> supplier){
        this.arm = arm;
        this.wrist = wrist;
        this.targetSelection = targetSelection;
        this.targetSupplier = supplier;
    }

    @Override
    public void initialize() {
        Type type = targetSupplier.get().getType();
        Height height = targetSupplier.get().getHeight();

        System.out.println("score position cmd " + type + " " + height);
        
        if (type == Type.CONE) {
            // move wrist for cone
            wrist.setTargetAngle(Wrist.SCORE_CONE);

            // extend telescope based on target selection height
            // raise one or both pistons according to height as mid doesnt need 2
            if (height == Height.HIGH) {
                arm.raiseBothPistons();
                arm.setTelescopePosition(Arm.HIGH_CONE);
            } else if (height == Height.MID) {
                arm.raiseBothPistons();
                arm.setTelescopePosition(Arm.MID_CONE);
            }
        } else if (type == Type.CUBE) {
            // lower arm for cubes in case its raised for some reason
            arm.setTelescopePosition(0);
            arm.lowerArm();

            // move wrist to scoring position accordingly
            if (height == Height.HIGH) {
                wrist.setTargetAngle(Wrist.SCORE_CUBE_FORWARD);
            } else if (height == Height.MID) {
                wrist.setTargetAngle(Wrist.SCORE_CUBE_FORWARD);
            } else if (height == Height.LOW) {
                // transport position is conveniently also good for low cube backwards
                wrist.setTargetAngle(Wrist.TRANSPORT);
            } 
        }
    }
}
