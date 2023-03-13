package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmPosition;

public class Wrist extends SubsystemBase {
    CANSparkMax wristController;

    public SparkMaxLimitSwitch forwardLimit;
    public SparkMaxLimitSwitch reverseLimit;

    private SparkMaxAnalogSensor pot;

    private SparkMaxPIDController pidController;

    private Arm arm; 

    private static final double kF = 0;
    private static final double kP = 0.45;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double MAX_OUTPUT = 0.4;
    private static final double MIN_OUTPUT = -0.2;

    private static final double MIN_ANGLE = -50;
    private static final double MAX_ANGLE = 110;

    // Function to convert from potentiometer volts to arm degrees above horizontal, obtained experimentally
    // Slope: degrees per volt
    // Constant: the degrees value at volts = 0
    private static final double VOLTS_TO_DEGREES_SLOPE = 66.0198;
    private static final double VOLTS_TO_DEGREES_CONSTANT = -93.6532;

    // Motor voltage required to hold arm up at horizontal
    // 0.075 is the experimentally determined motor percentage that does that, so convert % to volts:
    private static final double GRAVITY_COMPENSATION = 0.075 * 12;

    // Target angle and volts
    // Angle is relative to horizontal, so volts accounts for arm angle
    private double targetAngle;
    private double targetVolts;

    public static final double TRANSPORT = 116;
    //PLACEHOLDER VALUE
    public static final double LOADING_STATION = 0;
    public static final double GROUND = -40;
    // Placeholder for testing, needs bettter calibration
    public static final double SCORE_CONE = -15;
    public static final double SCORE_CUBE_BACKWARDS = 100;

    public Wrist(CANSparkMax controller, Arm arm) {
        wristController = controller;
        forwardLimit = wristController.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit = wristController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        this.arm = arm;

        pot = wristController.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

        pidController = wristController.getPIDController();

        // Use potentiometer for PID control
        pidController.setFeedbackDevice(pot);

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kF);
        pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

        // Initialize angle to where wrist is so it doesn't try to move on enable
        targetAngle = getCurrentAngle();
        targetVolts = convertAngleToVolts(targetAngle);

        controller.setSmartCurrentLimit(15, 35, 50);
        // TODO: Adjust ramp rate for best performance/jerk tradeoff
        controller.setClosedLoopRampRate(1);

    }

    /**
     * 
     * @return current wrist angle relative to horizontal (degrees)
     */
    public double getCurrentAngle() {
        double potVoltage = pot.getPosition();
        return potVoltage * VOLTS_TO_DEGREES_SLOPE + VOLTS_TO_DEGREES_CONSTANT + arm.getCurrentAngle();
    }

    public double getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * 
     * @param angle desired degrees above horizontal
     */
    public void setTargetAngle(double angle) { // abc1239+10=21 road work ahead, i sure hope it does. David was here.......
        if (angle <= MIN_ANGLE) {
            this.targetAngle = MIN_ANGLE;
        } else if (angle >= MAX_ANGLE) {
            this.targetAngle = MAX_ANGLE;
        } else {
            this.targetAngle = angle;
        }

        this.targetVolts = convertAngleToVolts(this.targetAngle);
    }

    /**
     * 
     * @param angle desired angle above horizontal (degrees)
     * @return pot position at this angle. this accounts for current arm angle
     */
    private double convertAngleToVolts(double angle) {
        // I hate algebra
        return angle * (1 / VOLTS_TO_DEGREES_SLOPE) - VOLTS_TO_DEGREES_CONSTANT * (1 / VOLTS_TO_DEGREES_SLOPE);
    }

    /**
     * 
     * @return  motor volts needed to "cancel out" gravity: (Gravity compensation constant) * cos(current angle)
     */
    private double getArbitraryFeedforward() {
        return GRAVITY_COMPENSATION * Math.cos(Math.toRadians(getCurrentAngle()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("wrist target deg horizontal", targetAngle);
        SmartDashboard.putNumber("wrist target pot volts", targetVolts);
        SmartDashboard.putNumber("wrist AFF", getArbitraryFeedforward());

        SmartDashboard.putNumber("arm curent", arm.getCurrentAngle());

        SmartDashboard.putNumber("wrist current deg horizontal", getCurrentAngle());
        SmartDashboard.putNumber("wrist current pot volts", pot.getPosition());


        // Let gravity lower arm to ground instead of slamming:
        // Stop pidcontroller if target angle is low, and arm is low enough to fall naturally
        if (getCurrentAngle() < -25 && targetAngle < -25) {
            wristController.set(0);
        } else if (getCurrentAngle() > 110 && targetAngle > 115) {
            wristController.set(0);
        } else {
            // Otherwise, continuously set wrist pid to target angle (must be continuous to update feedforward as angle changes)
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward());
        }
    }
}
