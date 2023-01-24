package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * Mapping and creation of hardware on the robot
 */
public class RobotMap {
    
    /**
     * Inner class to hold CAN ID constants.
     */
    private class CAN {
        // Drive controller CAN IDs
        private static final int LEFT_CONTROLLER_1 = 2;
        private static final int LEFT_CONTROLLER_2 = 3;
        private static final int LEFT_CONTROLLER_3 = 4;
        private static final int RIGHT_CONTROLLER_1 = 5;
        private static final int RIGHT_CONTROLLER_2 = 6;
        private static final int RIGHT_CONTROLLER_3 = 7;
    }

    // Left drive controllers
    public static final CANSparkMax leftController1 = new CANSparkMax(CAN.LEFT_CONTROLLER_1, MotorType.kBrushless);
    public static final CANSparkMax leftController2 = new CANSparkMax(CAN.LEFT_CONTROLLER_2, MotorType.kBrushless);
    public static final CANSparkMax leftController3 = new CANSparkMax(CAN.LEFT_CONTROLLER_3, MotorType.kBrushless);
    // Right drive controllers
    public static final CANSparkMax rightController1 = new CANSparkMax(CAN.RIGHT_CONTROLLER_1, MotorType.kBrushless);
    public static final CANSparkMax rightController2 = new CANSparkMax(CAN.RIGHT_CONTROLLER_2, MotorType.kBrushless);
    public static final CANSparkMax rightController3 = new CANSparkMax(CAN.RIGHT_CONTROLLER_3, MotorType.kBrushless);

    // Static initializer will be run on first reference to RobotMap
    static {
        leftController2.follow(leftController1);
        leftController3.follow(leftController1);
        
        rightController2.follow(rightController1);
        rightController3.follow(rightController1);

        leftController1.setInverted(true);
        leftController1.setOpenLoopRampRate(0.15);
        rightController1.setOpenLoopRampRate(0.15);

    }
}
