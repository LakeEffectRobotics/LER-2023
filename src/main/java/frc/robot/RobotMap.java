package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * Mapping and creation of hardware on the robot
 */
public class RobotMap {

    /**
     * Inner class to hold CAN ID constants.
     */
    private class CAN {
        // Drive controller CAN IDs
        private static final int LEFT_CONTROLLER_1 = 5;
        private static final int LEFT_CONTROLLER_2 = 6;
        private static final int LEFT_CONTROLLER_3 = 7;
        private static final int RIGHT_CONTROLLER_1 = 2;
        private static final int RIGHT_CONTROLLER_2 = 3;
        private static final int RIGHT_CONTROLLER_3 = 4;

        private static final int LEFT_CLAW_CONTROLLER = 9;
        private static final int RIGHT_CLAW_CONTROLLER = 10;
        
        // Wrist controller ID
        private static final int WRIST_CONTROLLER = 8;

        // Telescope controllers
        private static final int TELESCOPE_CONTROLLER_1 = 11;
        private static final int TELESCOPE_CONTROLLER_2 = 12;

    }

    /**
     * Inner class to hold pneumatic IDs
     */
    private class PCM {
        // Claw solenoids
        private static final int LEFT_CLAW_OPEN = 2;
        private static final int LEFT_CLAW_CLOSED = 6;

        private static final int RIGHT_CLAW_OPEN = 0;
        private static final int RIGHT_CLAW_CLOSED = 4;

        // Arm solenoid channels
        private static final int LEFT_ARM_UP = 3;
        private static final int LEFT_ARM_DOWN = 7;

        private static final int RIGHT_ARM_UP = 1;
        private static final int RIGHT_ARM_DOWN = 5;
    }

    // Left and right drive controllers
    public static final CANSparkMax leftController1 = new CANSparkMax(CAN.LEFT_CONTROLLER_1, MotorType.kBrushless);
    public static final CANSparkMax leftController2 = new CANSparkMax(CAN.LEFT_CONTROLLER_2, MotorType.kBrushless);
    public static final CANSparkMax leftController3 = new CANSparkMax(CAN.LEFT_CONTROLLER_3, MotorType.kBrushless);
    public static final CANSparkMax rightController1 = new CANSparkMax(CAN.RIGHT_CONTROLLER_1, MotorType.kBrushless);
    public static final CANSparkMax rightController2 = new CANSparkMax(CAN.RIGHT_CONTROLLER_2, MotorType.kBrushless);
    public static final CANSparkMax rightController3 = new CANSparkMax(CAN.RIGHT_CONTROLLER_3, MotorType.kBrushless);

    // Claw controllers
    public static final CANSparkMax leftClawController = new CANSparkMax(CAN.LEFT_CLAW_CONTROLLER,
            MotorType.kBrushless);
    public static final CANSparkMax rightClawController = new CANSparkMax(CAN.RIGHT_CLAW_CONTROLLER,
            MotorType.kBrushless);


    public static DoubleSolenoid leftClawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM.LEFT_CLAW_OPEN,
            PCM.LEFT_CLAW_CLOSED);
    public static DoubleSolenoid rightClawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM.RIGHT_CLAW_OPEN,
            PCM.RIGHT_CLAW_CLOSED);
            
    // Wrist controller
    public static final CANSparkMax wristController = new CANSparkMax(CAN.WRIST_CONTROLLER, MotorType.kBrushless);
    
    // Arm motor controllers
    public static final CANSparkMax telescopeController1 = new CANSparkMax(CAN.TELESCOPE_CONTROLLER_1,
            MotorType.kBrushless);
    public static final CANSparkMax telescopeController2 = new CANSparkMax(CAN.TELESCOPE_CONTROLLER_2,
            MotorType.kBrushless);

    // Arm solenoid
    public static DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM.LEFT_ARM_UP,
            PCM.LEFT_ARM_DOWN);

    public static DoubleSolenoid rightArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM.RIGHT_ARM_UP,
            PCM.RIGHT_ARM_DOWN);
    // Gyro
    public static final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Static initializer will be run on first reference to RobotMap
    static {
        // Drive motors
        leftController2.follow(leftController1);
        leftController3.follow(leftController1);

        rightController2.follow(rightController1);
        rightController3.follow(rightController1);

        rightController1.setInverted(true);

        leftController1.setClosedLoopRampRate(0.15);
        rightController1.setClosedLoopRampRate(0.15);

        leftController1.setOpenLoopRampRate(0.15);
        rightController1.setOpenLoopRampRate(0.15);

        // Claw motors
        leftClawController.setIdleMode(IdleMode.kBrake);
        rightClawController.setIdleMode(IdleMode.kBrake);
        
        // Wrist use brake mode
        wristController.setIdleMode(IdleMode.kBrake);

        // Arm motors
        telescopeController2.follow(telescopeController1);
        telescopeController1.setInverted(true);
    }
}
