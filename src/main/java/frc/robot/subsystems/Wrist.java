package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    CANSparkMax wristController;

    public Wrist(CANSparkMax controller) {
        this.wristController = controller;
    }

    /**
     * 
     * @param angle
     */
    public void goToAngle(double angle) {

    }
}
