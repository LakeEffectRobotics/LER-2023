package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase{
    /**
     * Sset drivetrain output
     * @param left [-1, 1]
     * @param right [-1, 1]
     */

     CANSparkMax leftLeadController;
     CANSparkMax rightLeadController;

    public Drivetrain(CANSparkMax leftLeadController, CANSparkMax rightLeadController) {
        this.leftLeadController = leftLeadController;
        this.rightLeadController = rightLeadController;
    }

    public void setOutput(double left, double right) {
        // Other controllers are followers
        leftLeadController.set(left);
        rightLeadController.set(right);
    }

    public void stop() {
        leftLeadController.set(0);
        rightLeadController.set(0);
    }
}
