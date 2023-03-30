package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.TargetSelection.Height;
import frc.robot.subsystems.TargetSelection.Node;
import frc.robot.subsystems.TargetSelection.Type;

public class ScoringPositionCommand  extends CommandBase {
    Arm arm;
    Wrist wrist;
    TargetSelection targetSelection;
    boolean auto;
    double wristAngle;
    double telescopePosition;
    double startTime;
    double timeout = 700;
    Type type;
    Height height;

    /**
     * for use in tele, with live targetselection
     * @param arm
     * @param wrist
     * @param targetSelection
     */
    public ScoringPositionCommand(Arm arm, Wrist wrist, TargetSelection targetSelection){
        this.arm = arm;
        this.wrist = wrist;
        this.targetSelection = targetSelection;
    }

    /**
     * for use in tele, with live targetselection
     * @param arm
     * @param wrist
     * @param targetSelection
     */
    public ScoringPositionCommand(Arm arm, Wrist wrist, Height height, Type type){
        this.arm = arm;
        this.wrist = wrist;
        this.type = type;
        this.height = height;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

        if (!auto) {
            height = targetSelection.getSelectedNode().getHeight();
            type = targetSelection.getSelectedNode().getType();
        }

        if (type == Type.CONE) {
            wristAngle = Wrist.SCORE_CONE;
            // move wrist for cone
        //    wrist.setTargetAngle(Wrist.SCORE_CONE);

            // extend telescope based on target selection height
            // raise one or both pistons according to height as mid doesnt need 2
            if (height == Height.HIGH) {
                telescopePosition = Arm.HIGH_CONE;
                arm.raiseBothPistons();
               // arm.setTelescopePosition(Arm.HIGH_CONE);
            } else if (height == Height.MID) {
                telescopePosition = Arm.MID_CONE;
                  arm.raiseBothPistons();
                //arm.setTelescopePosition(Arm.MID_CONE);
            } else if (height == Height.LOW) {
                telescopePosition = 0;
            }
        } else if (type == Type.CUBE) {
            // lower arm for cubes in case its raised for some reason
            telescopePosition = 0;
          
            // move wrist to scoring position accordingly
            if (height == Height.HIGH) {
                wristAngle = Wrist.SCORE_CUBE_FORWARD;
            } else if (height == Height.MID) {
                wristAngle = Wrist.SCORE_CUBE_FORWARD;
            } else if (height == Height.LOW) {
                // transport position is conveniently also good for low cube backwards
                wristAngle = Wrist.TRANSPORT;
            } 
        }

        wrist.setTargetAngle(wristAngle);
    }

    @Override
    public void execute(){
        // if wrist is alive and moves to the right position, continue doing stuff
        // otherwise keep waiting until wrist is close or timeout passes
        // OR, if timeout passes, continue even without wrist in position because something gone wrong
        if (!wrist.isWristDeadAgain) {
            if (Math.abs(wrist.getCurrentAngle() - wristAngle) < 10  || System.currentTimeMillis() - startTime < timeout) {
                arm.setTelescopePosition(telescopePosition);
            }
        } else {
            arm.setTelescopePosition(telescopePosition);
        }
    }

    @Override
    public boolean isFinished(){
        // if telescope and wrist are pretty much in position, end command
        // or if timeout passes, end command even if nothing is in position because something gone wrong!
        if (!wrist.isWristDeadAgain) {
            if ((Math.abs(arm.getTelescopePosition() - telescopePosition) < 3 && Math.abs(wrist.getCurrentAngle() - wristAngle) < 10)  || System.currentTimeMillis() - startTime < timeout) {
                return true;
            }
        } else {
            // if wrist is dead, ignore the wrist and only look for telescopy right position
            if (Math.abs(arm.getTelescopePosition() - telescopePosition) < 3) {
                return true;
            }
        }
        
        return false;
    }
}
