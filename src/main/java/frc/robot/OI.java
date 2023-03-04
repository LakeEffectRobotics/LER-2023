package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class holds configurations for the Operator Interface (so
 * joysticks/controllers)
 */
public class OI {

    /** Driver Station ports controllers are attached to */
    private static class PORTS {
        private static final int LEFT_STICK = 0;
        private static final int RIGHT_STICK = 1;
        private static final int XBOX_CONTROLLER = 2;
    }

    private static final Joystick leftJoystick = new Joystick(PORTS.LEFT_STICK);
    private static final Joystick rightJoystick = new Joystick(PORTS.RIGHT_STICK);

    /** Buttons on the driver sticks/controller */
    private static class DRIVER_MAP {
        private static final int AIM_BUTTON = 7;
        private static final int RIGHT_TRIGGER = 1;
        private static final int RIGHT_LEFT_BUTTON = 5;
        private static final int RIGHT_RIGHT_BUTTON = 4;
        private static final int RESET_POSE_BUTTON = 6;
        private static final int DISO_BUTTON = 2;
    }

    /** Buttons on the operator controller */
    private static class OPERATOR_MAP {
        private static final int OPEN_CLAW_BUTTON = XboxController.Button.kA.value;
        private static final int CLOSE_CLAW_BUTTON = XboxController.Button.kB.value;
        private static final int SPIN_INTAKE_BUTTON = XboxController.Button.kX.value;
        private static final int SPIT_OUT_BUTTON = XboxController.Button.kY.value;

    }

    private static final XboxController xboxController = new XboxController(PORTS.XBOX_CONTROLLER);

    public static final Trigger aimButton = new JoystickButton(rightJoystick, DRIVER_MAP.AIM_BUTTON);
    public static final Trigger resetPoseButton = new JoystickButton(xboxController, DRIVER_MAP.RESET_POSE_BUTTON);
    public static final JoystickButton curtisStraightButton = new JoystickButton(rightJoystick,
            DRIVER_MAP.RIGHT_TRIGGER);
    public static final JoystickButton curtisLeftButton = new JoystickButton(rightJoystick,
            DRIVER_MAP.RIGHT_LEFT_BUTTON);
    public static final JoystickButton curtisRightButton = new JoystickButton(rightJoystick,
            DRIVER_MAP.RIGHT_RIGHT_BUTTON);
    public static final JoystickButton dicoButton = new JoystickButton(rightJoystick, DRIVER_MAP.DISO_BUTTON);

    // Buttons
    public static final Trigger openClawButton = new JoystickButton(xboxController, OPERATOR_MAP.OPEN_CLAW_BUTTON);
    public static final Trigger closeClawButton = new JoystickButton(xboxController,
            OPERATOR_MAP.CLOSE_CLAW_BUTTON);

    public static final Trigger spinIntakeButton = new JoystickButton(xboxController,
            OPERATOR_MAP.SPIN_INTAKE_BUTTON);
    public static final Trigger spitOutButton = new JoystickButton(xboxController,
            OPERATOR_MAP.SPIT_OUT_BUTTON);

    // Supply processed drivetrain inputs
    public static DoubleSupplier leftDriveSupplier = () -> {
        double raw = leftJoystick.getY();
        // Todo: process raw input ?
        return processDriveInput(raw);
    };

    // Supply processed drivetrain inputs
    public static DoubleSupplier rightDriveSupplier = () -> {
        double raw = rightJoystick.getY();
        // Todo: process raw input ?
        return processDriveInput(raw);
    };

    private static double processDriveInput(double raw) {
        // TODO: Configure input processing to suit your liking
        if (Math.abs(raw) < 0.2)
            raw = 0;
        // raw = Math.pow(raw, [EXPONENT]);
        // raw *= [INPUT_SCALING];
        return -raw;
    }
}
