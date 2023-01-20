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
    public void setOutput(double left, double right) {
        // Other controllers are followers
        RobotMap.leftController1.set(-left);
        RobotMap.rightController1.set(right);
    }

    public void stop() {
        RobotMap.leftController1.set(0);
        RobotMap.rightController1.set(0);
    }
}
