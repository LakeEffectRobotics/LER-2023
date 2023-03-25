package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public CANSparkMax wristController;

    public SparkMaxLimitSwitch forwardLimit;
    public SparkMaxLimitSwitch reverseLimit;

    private SparkMaxAnalogSensor pot;

    private SparkMaxPIDController pidController;

    private Arm arm; 

    private static final double kF = 0;
    private static final double kP = 0.4;
    private static final double kI = 0.0000;

    private static final double kD = 0.1;
    private static final double MAX_OUTPUT = 0.26;
    private static final double MIN_OUTPUT = -0.20;

    private static final double MIN_ANGLE = -50;
    private static final double MAX_ANGLE = 122;

    // Function to convert from potentiometer volts to arm degrees above horizontal, obtained experimentally
    // Slope: degrees per volt
    // Constant: the degrees value at volts = 0
    private static final double VOLTS_TO_DEGREES_SLOPE = 70.5821;
    private static final double VOLTS_TO_DEGREES_CONSTANT = -106.185;

    // Motor voltage required to hold arm up at horizontal
    // 0.05 is the experimentally determined motor percentage that does that, so convert % to volts:
    private static final double GRAVITY_COMPENSATION = 0.055 * 12;

    // Target angle and volts
    // Angle is relative to horizontal, so volts accounts for arm angle
    private double targetAngle;
    private double targetVolts;

    public static final double TRANSPORT = 124;
    //PLACEHOLDER VALUE
    public static final double SINGLE_LOADING = -12;
    public static final double DOUBLE_LOADING = -60;
    public static final double GROUND = -48;
    // Placeholder for testing, needs bettter calibration
    public static final double SCORE_CONE = 0;
    public static final double SCORE_CUBE_BACKWARDS = 100;
    public static final double SCORE_CUBE_FORWARD = 14;

    public boolean isWristDeadAgain = false;

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
        pidController.setIAccum(0.05);
        pidController.setFF(kF);
        pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

        // Initialize angle to where wrist is so it doesn't try to move on enable
        targetAngle = getCurrentAngle();
        targetVolts = convertAngleToVolts(targetAngle);

        controller.setSmartCurrentLimit(15, 35, 50);
        // TODO: Adjust ramp rate for best performance/jerk tradeoff
        controller.setClosedLoopRampRate(1);

        SmartDashboard.putString("wrist dead?", "not yet!");
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

        this.targetVolts = convertAngleToVolts(this.targetAngle - arm.getCurrentAngle());
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

    /**
     * toggle wrist deadness; makes certain things go into wrist dead mode
     */
    public void wristDead() {
        isWristDeadAgain = !isWristDeadAgain;
        if (isWristDeadAgain) {
            wristController.setIdleMode(IdleMode.kCoast);
            SmartDashboard.putString("wrist dead?", "yes :(");
        } else {
            wristController.setIdleMode(IdleMode.kBrake);
            SmartDashboard.putString("wrist dead?", "not yet!");
        }
    }

    public void setMotors(double speed) {
        wristController.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("wrist target deg horizontal", targetAngle);
        SmartDashboard.putNumber("wrist target pot volts", targetVolts);

        SmartDashboard.putNumber("wrist current deg horizontal", getCurrentAngle());
        SmartDashboard.putNumber("wrist current pot volts", pot.getPosition());

        // if wrist is dead kill motor just in case
        if (isWristDeadAgain){
            wristController.set(0);
        } else if (arm.getCurrentAngle() > 10 && getCurrentAngle() > 90)  {
            // if its past vertical, spring pulls against so make AFF a tad more downward to help
            // since arm is up, it needs even more help
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward() - 1);
        }
        else if (getCurrentAngle() > 90) {
            // if its past vertical, spring pulls against so make AFF a tad more downward to help
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward() - 0.4);
        } 
        else if (getCurrentAngle() < 0 && targetAngle < -30) {
        // Let gravity lower arm to ground instead of slamming:
        // Stop pidcontroller if target angle is low, and arm is low enough to fall naturally
            wristController.setVoltage(0);
        } else if (getCurrentAngle() > 90 && targetAngle > 90) {
            // also stop pid if its far back enought to fall onto hardstop alone
            // try adding back pid cuz slam
            wristController.setVoltage(getArbitraryFeedforward() + 0.4);;
        }  else {
            // Otherwise, continuously set wrist pid to target angle (must be continuous to update feedforward as angle changes)
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward());
        }
    }
}
