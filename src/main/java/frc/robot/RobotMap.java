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
    private static final CANSparkMax leftController1 = new CANSparkMax(CAN.LEFT_CONTROLLER_1, MotorType.kBrushless);
    private static final CANSparkMax leftController2 = new CANSparkMax(CAN.LEFT_CONTROLLER_2, MotorType.kBrushless);
    private static final CANSparkMax leftController3 = new CANSparkMax(CAN.LEFT_CONTROLLER_3, MotorType.kBrushless);
    // Right drive controllers
    private static final CANSparkMax rightController1 = new CANSparkMax(CAN.RIGHT_CONTROLLER_1, MotorType.kBrushless);
    private static final CANSparkMax rightController2 = new CANSparkMax(CAN.RIGHT_CONTROLLER_2, MotorType.kBrushless);
    private static final CANSparkMax rightController3 = new CANSparkMax(CAN.RIGHT_CONTROLLER_3, MotorType.kBrushless);

    // Group left and right controllers
    public static final MotorControllerGroup leftDriveControllers = new MotorControllerGroup(leftController1, leftController2, leftController3);
    public static final MotorControllerGroup rightDriveControllers = new MotorControllerGroup(rightController1, rightController2,rightController3);

    // Static initializer will be run on first reference to RobotMap
    static {
        // It's a bit ambiguous what config belongs here vs elsewhere, but a good rule of thumb is that anything that 
        // isn't implicitly part of a motor's name (so PID constants, ramp settings, etc) belong in subsystems
    }
}
