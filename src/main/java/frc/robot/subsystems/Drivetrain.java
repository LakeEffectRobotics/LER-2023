package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    
    MotorControllerGroup leftControllers;
    MotorControllerGroup rightControllers;

    public Drivetrain(MotorControllerGroup leftControllers, MotorControllerGroup rightControllers) {
        this.leftControllers = leftControllers;
        this.rightControllers = rightControllers;
    }

    /**
     * Sset drivetrain output
     * @param left [-1, 1]
     * @param right [-1, 1]
     */
    public void setOutput(double left, double right) {
        leftControllers.set(-left);
        rightControllers.set(right);
    }

    public void stop() {
        leftControllers.set(0);
        rightControllers.set(0);
    }
}
