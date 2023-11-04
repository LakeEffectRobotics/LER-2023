package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public CANSparkMax wristController;

    private SparkMaxAnalogSensor pot;

    private SparkMaxPIDController pidController;

    private Arm arm; 

    private static final double kF = 0;
    private static final double kP = 0.4;
    private static final double kI = 0;

    private static final double kD = 0.5;
    private static final double MAX_OUTPUT = 0.5;
    private static final double MIN_OUTPUT = -0.2;

    private static final double MIN_ANGLE = -70;
    private static final double MAX_ANGLE = 122;

    // Function to convert from potentiometer volts to arm degrees above horizontal, obtained experimentally
    // Slope: degrees per volt
    // Constant: the degrees value at volts = 0
    private static final double VOLTS_TO_DEGREES_SLOPE = 70.5821;
    private static final double VOLTS_TO_DEGREES_CONSTANT = -106.185;

    // Motor voltage required to hold arm up at horizontal
    // 0.05 is the experimentally determined motor percentage that does that, so convert % to volts:
    private static final double GRAVITY_COMPENSATION = 0.04 * 12;

    // Target angle and volts
    // Angle is relative to horizontal, so volts accounts for arm angle
    private double targetAngle;
    private double targetVolts;

    public static final double TRANSPORT = 110;
    //PLACEHOLDER VALUE
    public static final double SINGLE_LOADING = -30;
    public static final double DOUBLE_LOADING = -60;
    // lower than actually needed so P drives it down faster
    public static final double GROUND = -70;
    // Placeholder for testing, needs bettter calibration
    public static final double SCORE_HIGH_CONE = 5;
    public static final double SCORE_MID_CONE = -30;

    public static final double SCORE_CUBE_BACKWARDS = 100;    
    public static final double SCORE_MID_CUBE_FORWARD = -45;
    public static final double SCORE_HIGH_CUBE_FORWARD = -20;

    public static final double SCORE_CUBE_FORWARD_NO_ARM = 0;

    public boolean isWristDeadAgain = false;

    private ShuffleboardTab tab = Shuffleboard.getTab("my favourite tab");
    private GenericEntry wristDeadShuffle = tab
        .add("wrist dead?", "not yet!")
        .withPosition(6, 0)
        .getEntry();

    private GenericEntry targetAngleShuffle;
    private GenericEntry targetPotShuffle;

    private GenericEntry currentAngleShuffle;
    private GenericEntry currentPotShuffle;
    

    public Wrist(CANSparkMax controller, Arm arm) {
        wristController = controller;

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
        targetVolts = convertAngleToVolts(targetAngle - arm.getCurrentAngle());

        controller.setSmartCurrentLimit(24, 40, 5);
        // TODO: Adjust ramp rate for best performance/jerk tradeoff
        controller.setClosedLoopRampRate(1);

        targetAngleShuffle = tab
            .add("wrist target angle", targetAngle)
            .withPosition(3, 0)
            .getEntry();

        targetPotShuffle = tab
            .add("wrist target pot volts", targetVolts)
            .withPosition(4, 0)
            .getEntry();

        currentAngleShuffle = tab
            .add("wrist current angle", getCurrentAngle())
            .withPosition(3, 1)
            .getEntry();

        currentPotShuffle = tab
            .add("wrist current pot volts", pot.getPosition())
            .withPosition(4, 1)
            .getEntry();
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
            wristDeadShuffle.setString("yes :(");
        } else {
            wristController.setIdleMode(IdleMode.kBrake);
            wristDeadShuffle.setString("not yet!");
        }
    }

    public void setMotors(double speed) {
        wristController.set(speed);
    }

    @Override
    public void periodic() {
      //  SmartDashboard.putNumber("Current", wristController.getOutputCurrent());
        //SmartDashboard.putNumber("Output", wristController.getAppliedOutput());
       // SmartDashboard.putNumber("Position", getCurrentAngle());
        //SmartDashboard.putNumber("Target", targetAngle);

        currentAngleShuffle.setDouble(getCurrentAngle());
        currentPotShuffle.setDouble(pot.getPosition());

        targetAngleShuffle.setDouble(targetAngle);
        targetPotShuffle.setDouble(targetVolts);

        // if wrist is dead kill motor just in case
        if (isWristDeadAgain){
            wristController.set(0);
        } else if (arm.getCurrentAngle() > 10 && getCurrentAngle() > 90 && targetAngle < 150) {
            //System.out.println("1");
            // if its past vertical, spring pulls against so make AFF a tad more downward to help
            // since arm is up, it needs even more help
            pidController.setOutputRange(-0.3, MAX_OUTPUT, 0);
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward() - 0.5);
        } else if (arm.getCurrentAngle() > 10 && getCurrentAngle() <-20 && targetAngle < -20) {
            // if arm is up and we are at the bottom hard stop, do nothing
            wristController.set(0);
        } else if (getCurrentAngle() > 70 && targetAngle > 100) {
            //System.out.println("12");

            // also stop pid if its far back enought to fall onto hardstop alone
            // try adding back pid cuz slam
            wristController.setVoltage(0);
        } else if (getCurrentAngle() > 90 && targetAngle < 100) {

            // if its past vertical and we want to go down, spring pulls against so make AFF a tad more downward to help
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward() - 0.5);
        }  else if (getCurrentAngle() < -50 && targetAngle < -50) {
           // System.out.println("14");

        // Let gravity lower arm to ground instead of slamming:
        // Stop pidcontroller if target angle is low, and arm is low enough to fall naturally
            wristController.setVoltage(0);
        }  else if (getCurrentAngle() < 90 && targetAngle < -40) {
           // System.out.println("16");

            // yes ff on the way down and make it a bit slower
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward());
        } else {
           // System.out.println("15");
            // Otherwise, continuously set wrist pid to target angle (must be continuous to update feedforward as angle changes)
            pidController.setReference(targetVolts, ControlType.kPosition, 0, getArbitraryFeedforward());
        }
    }
}
