package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Claw.Position;
import frc.robot.subsystems.TargetSelection.Type;

public class SpinClawCommand extends CommandBase {
    
    /**
     * Claw spin direction
     */
    public enum Direction {
        IN(-1), 
        OUT(1);

        /**
         * Coefficient to get motor direction from always-positive speed
         */
        private int sign;

        private Direction(int sign){
            this.sign = sign;
        }
        /**
         * Get the sign (+/- 1) to get motor direction from always-positive speed
         */
        int getSign(){
            return sign;
        }
    }

    Claw claw;
    DoubleSupplier speedSupplier;
    TargetSelection targetSelection;
    Type type;
    boolean auto;
    Direction direction;

    /**
     * Create a new SpinIntakeCommand using live target selection
     * @param claw Claw subsystem
     * @param direction Direction for +ve spin
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins specified direction
     */
    public SpinClawCommand(Claw claw, Direction direction, DoubleSupplier speedSupplier, TargetSelection targetSelection) {
        this.claw = claw;
        this.speedSupplier = () -> speedSupplier.getAsDouble() * direction.sign;
        this.targetSelection = targetSelection;
        this.auto = false;
        this.direction = direction;
    }

    /**
     * Create a new SpinIntakeCommand for auto usage, passing in a type
     * @param claw Claw subsystem
     * @param direction Direction for +ve spin
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins specified direction
     */
    public SpinClawCommand(Claw claw, Direction direction, DoubleSupplier speedSupplier, Type type) {
        this.claw = claw;
        this.speedSupplier = () -> speedSupplier.getAsDouble() * direction.sign;
        this.auto = true;
        this.type = type;
        this.direction = direction;
    }


    @Override
    public void initialize() {
        if (!auto) {
            this.type = targetSelection.getSelectedNode().getType();
        }
    }

    @Override
    public void execute() {
        claw.setSpeed(speedSupplier.getAsDouble());

        // if direction is intaking, care about limit switch stuff
        if (direction == Direction.IN) {
            if (type == Type.CUBE) {
                // if targeting cubes, stop spinning when limit switch pressed
                // this is already happening with the hardware but might as well add here too
                if (claw.GetLimitPressed()) {
                    claw.setSpeed(0);
                }
            } else if (type == Type.CONE) {
                if (claw.GetLimitPressed()) {
                    claw.setSpeed(0);
                    claw.setPosition(Position.CLOSED);
                }
            }
        }

        // show on shuffleboard if limit switch pressed
        if (claw.GetLimitPressed()) {
            claw.limitswitchShuffle.setBoolean(true);
        } else {
            claw.limitswitchShuffle.setBoolean(false);
        }

    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
