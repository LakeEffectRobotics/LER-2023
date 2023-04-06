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
    double timeout = 1500;
    Type type;
    Height height;

    /**
     * for use in tele, with live targetselection
     * @param arm
     * @param wrist
     * @param targetSelection
     */
    public ScoringPositionCommand(Arm arm, Wrist wrist, TargetSelection targetSelection){
        addRequirements(arm);
        this.arm = arm;
        this.wrist = wrist;
        this.targetSelection = targetSelection;
        this.auto = false;
    }

    /**
     * for use in auto using a given node type/height
     * @param arm
     * @param wrist
     * @param targetSelection
     */
    public ScoringPositionCommand(Arm arm, Wrist wrist, Height height, Type type){
        addRequirements(arm);
        this.arm = arm;
        this.wrist = wrist;
        this.type = type;
        this.height = height;
        this.auto = true;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

        if (!auto) {
            height = targetSelection.getSelectedNode().getHeight();
            type = targetSelection.getSelectedNode().getType();
        }

        if (type == Type.CONE) {
            wristAngle = Wrist.SCORE_HIGH_CONE;

            // extend telescope based on target selection height
            // raise one or both pistons according to height as mid doesnt need 2
            switch(height){
                case HIGH:
                    telescopePosition = Arm.HIGH_CONE;
                    arm.raiseBothPistons();
                    break;
                case MID:
                    telescopePosition = Arm.MID_CONE;
                    arm.raiseBothPistons();
                    break;
                case LOW:
                    telescopePosition = 0;
                    break;
            }
        } else if (type == Type.CUBE) {
            // if wrist is ALIVE, dont need to use telescope (? unless it turns out to be useful ? ??)
            // so, just move wrist
            if (!wrist.isWristDeadAgain) {
                switch(height){
                    case HIGH:
                        telescopePosition = Arm.ALL_CUBE;
                        wristAngle = Wrist.SCORE_HIGH_CUBE_FORWARD;
                        arm.raiseBothPistons();
                        break;
                    case MID:
                        telescopePosition = Arm.ALL_CUBE;
                        wristAngle = Wrist.SCORE_MID_CUBE_FORWARD;
                        arm.raiseBothPistons();
                        break;
                    case LOW:
                        // transport position is conveniently also good for low cube backwards
                        telescopePosition = 0;
                        wristAngle = Wrist.TRANSPORT;
                        break;
                }
            } else {
                // if wrist is dead, just move telescope
                switch(height){
                    case HIGH:
                        telescopePosition = Arm.HIGH_CUBE_DEAD;
                        arm.raiseBothPistons();
                        break;
                    case MID:
                        telescopePosition = Arm.MID_CUBE_DEAD;
                        arm.raiseBothPistons();
                        break;
                    case LOW:
                        telescopePosition = 0;
                        break;
                }
            }
        }
        wrist.setTargetAngle(wristAngle);

        // help wrist get to position when it starts way back with arm up
        // probably temporary until motor alives more?
        if (auto) {
            wrist.setMotors(-1);
        }
    }

    @Override
    public void execute(){
        // if wrist is alive and moves to the right position, continue doing stuff
        // otherwise keep waiting until wrist is close or timeout passes
        // OR, if timeout passes, continue even without wrist in position because something gone wrong
        if (!wrist.isWristDeadAgain) {
            if (Math.abs(wrist.getCurrentAngle() - wristAngle) < 10  || System.currentTimeMillis() - startTime > timeout) {
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
            if ((Math.abs(arm.getTelescopePosition() - telescopePosition) < 3 && Math.abs(wrist.getCurrentAngle() - wristAngle) < 10)  || System.currentTimeMillis() - startTime > timeout) {
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
