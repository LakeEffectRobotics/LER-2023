package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax telescopeController1;
    CANSparkMax telescopeController2;

    private SparkMaxLimitSwitch forwardLimit;
    private SparkMaxLimitSwitch reverseLimit;

    DoubleSolenoid leftSolenoid;
    DoubleSolenoid rightSolenoid;

    ArmPosition currentPosition;

    SparkMaxPIDController pidController;

    private static final double kF = 0;
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double MAX_OUTPUT = 1;
    private static final double MIN_OUTPUT = -1;

    public Arm(CANSparkMax controller1, CANSparkMax controller2, DoubleSolenoid leftSolenoid,
            DoubleSolenoid rightSolenoid) {
        this.telescopeController1 = controller1;
        this.telescopeController2 = controller2;
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;

        // set PID constants
        pidController = telescopeController1.getPIDController();

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kF);
        pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

        // Limit switches for telescope (other controller is follower)
        forwardLimit = telescopeController1.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit = telescopeController1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    // Arm piston positions: up, down, neutral
    public enum ArmPosition {
        UP(DoubleSolenoid.Value.kForward),
        DOWN(DoubleSolenoid.Value.kReverse),
        NEUTRAL(DoubleSolenoid.Value.kOff);

        private DoubleSolenoid.Value value;

        private ArmPosition(DoubleSolenoid.Value value) {
            this.value = value;
        }

        public DoubleSolenoid.Value getValue() {
            return value;
        }
    }

    /**
     * Extend both arm pistons
     */
    public void raiseArm() {
        currentPosition = ArmPosition.UP;

        leftSolenoid.set(ArmPosition.UP.value);
        rightSolenoid.set(ArmPosition.UP.value);
    }

    /**
     * Release both arm pistons
     */
    public void lowerArm() {
        currentPosition = ArmPosition.DOWN;

        leftSolenoid.set(ArmPosition.DOWN.value);
        rightSolenoid.set(ArmPosition.DOWN.value);
    }

    /**
     * 
     * @return Current arm position (up, down, neutral)
     */
    public ArmPosition getArmPosition() {
        return currentPosition;
    }

    /**
     * Extend/retract telescope
     * 
     * @param position -1 to 1
     */
    public void setTelescopePosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }
}
