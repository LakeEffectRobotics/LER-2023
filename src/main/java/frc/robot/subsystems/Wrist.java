package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    CANSparkMax wristController;

    public SparkMaxLimitSwitch forwardLimit;
    public SparkMaxLimitSwitch reverseLimit;

    private SparkMaxAnalogSensor pot;

    private SparkMaxPIDController pidController;

    private static final double kF = 0;
    private static final double kP = 0.45;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double MAX_OUTPUT = 0.4;
    private static final double MIN_OUTPUT = -0.2;

    // Angle of the arm relative to horizontal ground (degrees)
    // Currently a constant as arm prototype is stationary
    private static final double ARM_ANGLE = 1.6;

    // Function to convert from potentiometer volts to arm degrees above horizontal
    // Slope: degrees per volt
    // Constant: the degrees value at volts = 0
    private static final double VOLTS_TO_DEGREES_SLOPE = 67.95;
    private static final double VOLTS_TO_DEGREES_CONSTANT = -82.16;

    // Voltage required to hold arm up at horizontal
    // 0.075 is the experimentally determined motor percentage that does that
    private static final double GRAVITY_COMPENSATION = 0.075 * 12;

    // Wrist angle
    private double targetAngle;

    public static final double TRANSPORT = 80;
    //PLACEHOLDER VALUE
    public static final double LOADING_STATION = 0;
    public static final double GROUND = -40;
    public static final double SCORE_CONE = 0;

    public Wrist(CANSparkMax controller) {
        wristController = controller;
        forwardLimit = wristController.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit = wristController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

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
    }

    /**
     * 
     * @return current wrist angle relative to horizontal (degrees)
     */
    public double getCurrentAngle() {
        double potVoltage = pot.getPosition();
        return potVoltage * VOLTS_TO_DEGREES_SLOPE + VOLTS_TO_DEGREES_CONSTANT + ARM_ANGLE;
    }

    public double convertAngleToVolts(double angle) {
        // I hate algebra
        return angle * (1 / VOLTS_TO_DEGREES_SLOPE) - VOLTS_TO_DEGREES_CONSTANT * (1 / VOLTS_TO_DEGREES_SLOPE);
    }

    /**
     * 
     * @param angle desired degrees above horizontal
     */
    public void setTargetAngle(double angle) { // abc1239+10=21 road work ahead, i sure hope it does. David was here.......
        this.targetAngle = angle;
    }

    private double getArbitraryFeedforward() {
        // Return volts needed to "cancel out" gravity: (Gravity compensation constant) * cos(current angle)
        return GRAVITY_COMPENSATION * Math.cos(Math.toRadians(getCurrentAngle()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("wrist target angle", targetAngle);
        SmartDashboard.putNumber("wrist target volts", convertAngleToVolts(targetAngle));
        SmartDashboard.putNumber("wrist AFF", getArbitraryFeedforward());

        SmartDashboard.putNumber("wrist current angle", getCurrentAngle());
        SmartDashboard.putNumber("wrist current volts", pot.getPosition());

        // Let gravity lower arm to ground instead of slamming:
        // Stop pidcontroller if target angle is low, and arm is low enough to fall naturally
        if (getCurrentAngle() < 0 && targetAngle < -15) {
            wristController.set(0);
        } else {
            // Otherwise, continuously set wrist pid to target angle (must be continuous to update feedforward as angle changes)
            pidController.setReference(convertAngleToVolts(targetAngle), ControlType.kPosition, 0, getArbitraryFeedforward());
        }
    }
}
