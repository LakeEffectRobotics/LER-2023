package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class holds configurations for the Operator Interface (so joysticks/controllers)
 */
public class OI {
    
    /** Driver Station ports controllers are attached to */
    private static class PORTS {
        private static final int LEFT_STICK = 0;
        private static final int RIGHT_STICK = 1;
    }

    private static final Joystick leftJoystick = new Joystick(PORTS.LEFT_STICK);
    private static final Joystick rightJoystick = new Joystick(PORTS.RIGHT_STICK);

    /** Buttons on the driver sticks/controller */
    private static class DRIVER_MAP {
        private static final int RIGHT_TRIGGER = 0;
        private static final int RIGHT_LEFT_BUTTON = 4;
        private static final int RIGHT_RIGHT_BUTTON = 5;
    }

    /** Buttons on the operator controller */
    private static class OPERATOR_MAP {
        
    }

    public static final JoystickButton curtisStraightButton = new JoystickButton(rightJoystick, DRIVER_MAP.RIGHT_TRIGGER);
    public static final JoystickButton curtisLeftButton = new JoystickButton(rightJoystick, DRIVER_MAP.RIGHT_LEFT_BUTTON);
    public static final JoystickButton curtisRightButton = new JoystickButton(rightJoystick, DRIVER_MAP.RIGHT_RIGHT_BUTTON);

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

    private static double processDriveInput(double raw){
        // TODO: Configure input processing to suit your liking
        if(Math.abs(raw) < 0.1) raw = 0;
        // raw = Math.pow(raw, [EXPONENT]);
        // raw *= [INPUT_SCALING];
        return raw;
    }
}
