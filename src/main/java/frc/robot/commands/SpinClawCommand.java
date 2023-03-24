package frc.robot.commands;

import java.util.function.DoubleSupplier;

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

    /**
     * Create a new SpinIntakeCommand, private because we require a direction
     * @param claw Claw subsystem
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins out, -ve spins in
     */
    private SpinClawCommand(Claw claw, DoubleSupplier speedSupplier, TargetSelection targetSelection) {
        addRequirements(claw);
        this.claw = claw;
        this.speedSupplier = speedSupplier;
        this.targetSelection = targetSelection;
    }

    /**
     * Create a new SpinIntakeCommand
     * @param claw Claw subsystem
     * @param direction Direction for +ve spin
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins specified direction
     */
    public SpinClawCommand(Claw claw, Direction direction, DoubleSupplier speedSupplier, TargetSelection targetSelection) {
        // Call normal constructor, but wrap supplier with multiplication by direction's sign
        this(claw, () -> speedSupplier.getAsDouble() * direction.sign, targetSelection);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setSpeed(speedSupplier.getAsDouble());

        if (targetSelection.getSelectedNode().getType() == Type.CUBE) {
            // if targeting cubes, stop spinning when limit switch pressed
            // this is already happening with the hardware but might as well add here too
            if (claw.GetLimitPressed()) {
                claw.setSpeed(0);
            }
        } else if (targetSelection.getSelectedNode().getType() == Type.CONE) {
            if (claw.GetLimitPressed()) {
                claw.setSpeed(0);
                claw.setPosition(Position.CLOSED);
            }
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
