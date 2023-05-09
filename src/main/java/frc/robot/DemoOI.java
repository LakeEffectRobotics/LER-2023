package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DemoOI {
    
    private static final int DEMO_CONTROLLER = 4;

    private static class DEMO_MAP {
        private static final int DISABLE_BUTTON = XboxController.Button.kStart.value;
        private static final int DISCO_BUTTON = XboxController.Button.kBack.value;
        private static final int OPEN_CLAW_BUTTON = XboxController.Button.kLeftBumper.value;
        private static final int CLOSE_CLAW_BUTTON = XboxController.Button.kRightBumper.value;

        // NOTE: This is expected to be an axis. If it is changed to a button, then modify the stuff acordingly
        private static final int SPIN_IN_TRIGGER = XboxController.Axis.kLeftTrigger.value;
        private static final int SPIT_OUT_TRIGGER = XboxController.Axis.kRightTrigger.value;

        // XBoxController DPAD Buttons
        private static final int UP_SELECTION_BUTTON = 180;
        private static final int RIGHT_SELECTION_BUTTON = 90;
        private static final int DOWN_SELECTION_BUTTON = 0;
        private static final int LEFT_SELECTION_BUTTON = 270;
    }

    private static final XboxController demoController = new XboxController(DEMO_CONTROLLER);

    /**
     * The threshold that must be met before an xbox is trigger is considered "pressed". Used to bind command so triggers
     */
    private static final double XBOX_TRIGGER_THRESHOLD = 0.2;

    public static final Trigger disableButton = new JoystickButton(demoController, DEMO_MAP.DISABLE_BUTTON);
    public static final Trigger discoButton = new JoystickButton(demoController, DEMO_MAP.DISCO_BUTTON);
    public static final Trigger openClawButton = new JoystickButton(demoController, DEMO_MAP.OPEN_CLAW_BUTTON);
    public static final Trigger closeClawButton = new JoystickButton(demoController, DEMO_MAP.CLOSE_CLAW_BUTTON);

    public static final Trigger spinInButton = new Trigger(() -> demoController.getRawAxis(DEMO_MAP.SPIN_IN_TRIGGER) >= XBOX_TRIGGER_THRESHOLD);
    public static final Trigger spitOutButton = new Trigger(() -> demoController.getRawAxis(DEMO_MAP.SPIT_OUT_TRIGGER) >= XBOX_TRIGGER_THRESHOLD);
    
    public static final Trigger upSelectionButton = new POVButton(demoController, DEMO_MAP.UP_SELECTION_BUTTON);
    public static final Trigger rightSelectionButton = new POVButton(demoController, DEMO_MAP.RIGHT_SELECTION_BUTTON);
    public static final Trigger downSelectionButton = new POVButton(demoController, DEMO_MAP.DOWN_SELECTION_BUTTON);
    public static final Trigger leftSelectionButton = new POVButton(demoController, DEMO_MAP.LEFT_SELECTION_BUTTON);


    public static DoubleSupplier leftDriveSupplier = () -> {
        double raw = demoController.getLeftY();
        return processDriveInput(raw);
    };

    public static DoubleSupplier rightDriveSupplier = () -> {
        double raw = demoController.getLeftX();
        return processDriveInput(raw);
    };

    private static double processDriveInput(double raw) {
        // TODO: Configure input processing to suit your liking
        if (Math.abs(raw) < 0.1) {
            raw = 0;
        }
        
        // Signum function is -1 for x < 0, 1 for x > 0
        raw = Math.pow(raw, 2) * Math.signum(raw);
       // raw *= 0.8;
        return -raw;
    }

    /**
     * Operator-supplied intake spin speed
     */
    public static DoubleSupplier clawInSpeedSupplier = () -> {
        return Math.pow(demoController.getRawAxis(DEMO_MAP.SPIN_IN_TRIGGER), 2) * 0.6;
    };

    /**
     * Operator-supplied outtake spin speed
     */
    public static DoubleSupplier clawOutSpeedSupplier = () -> {
        return Math.pow(demoController.getRawAxis(DEMO_MAP.SPIT_OUT_TRIGGER), 2);
    };

    public static DoubleSupplier manualMoveWristSupplier = () -> {
        return -demoController.getRightY();
    };

    public static DoubleSupplier manualMoveArmSupplier = () -> {
        return processDriveInput(demoController.getRightX());
    };
    
}
