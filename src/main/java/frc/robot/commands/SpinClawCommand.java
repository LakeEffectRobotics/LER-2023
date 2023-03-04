package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

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

    /**
     * Create a new SpinIntakeCommand
     * @param claw Claw subsystem
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins out, -ve spins in
     */
    public SpinClawCommand(Claw claw, DoubleSupplier speedSupplier) {
        addRequirements(claw);
        this.claw = claw;
        this.speedSupplier = speedSupplier;
    }

    /**
     * Create a new SpinIntakeCommand
     * @param claw Claw subsystem
     * @param direction Direction for +ve spin
     * @param speedSupplier Double supplier for intake speed, in %. +ve spins specified direction
     */
    public SpinClawCommand(Claw claw, Direction direction, DoubleSupplier speedSupplier) {
        // Call normal constructor, but wrap supplier with multiplication by direction's sign
        this(claw, () -> speedSupplier.getAsDouble() * direction.sign);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setSpeed(0);
    }
}
