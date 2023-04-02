package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax telescopeController1;
    CANSparkMax telescopeController2;

    DoubleSolenoid leftSolenoid;
    DoubleSolenoid rightSolenoid;

    ArmPosition pistonsCurrentPosition = ArmPosition.UP;

    public double telescopeTargetPosition = 0;

    SparkMaxPIDController pidController;

    private static final double kP = 1.5;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double MAX_OUTPUT = 0.6;
    private static final double MIN_OUTPUT = 0.001;

    public static final double MAX_POSITION = 23;
    public static final double MIN_POSITION = 0;

    public static final double HIGH_CONE = 20;
    public static final double MID_CONE = 9;

    // FOR WRIST DEAD MODE
    public static final double HIGH_CUBE_DEAD = 21;
    public static final double MID_CUBE_DEAD = 15;

    // TODO: calibrate value, then see if using telescope for all mid/high cubes is effective
    public static final double ALL_CUBE = 15;

    public static final double DOUBLE_LOADING = 23;

    public Arm(CANSparkMax controller1, CANSparkMax controller2, DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
        this.telescopeController1 = controller1;
        this.telescopeController2 = controller2;
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;

        // set PID constants
        pidController = telescopeController1.getPIDController();

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

       telescopeController1.getEncoder().setPosition(0);
       telescopeController2.getEncoder().setPosition(0);
    }

    // Arm piston positions: up, down, neutral
    public enum ArmPosition {
        UP(DoubleSolenoid.Value.kForward, 34),
        DOWN(DoubleSolenoid.Value.kReverse, 0);

        private DoubleSolenoid.Value value;
        private double angle;

        private ArmPosition(DoubleSolenoid.Value value, double angle) {
            this.value = value;
            this.angle = angle;
        }

        public DoubleSolenoid.Value getValue() {
            return value;
        }

        public double getAngle() {
            return angle;
        }
    }

    /**
     * Extend both arm pistons
     */
    public void raiseBothPistons() {
        pistonsCurrentPosition = ArmPosition.UP;

        leftSolenoid.set(ArmPosition.UP.value);
        rightSolenoid.set(ArmPosition.UP.value);
        SmartDashboard.putString("arm up?", "YES!");
    }

    public void raiseOnePiston() {
        // randomize which piston
        if (Math.random() < 0.5) {
            leftSolenoid.set(ArmPosition.UP.value); 
        } else {
            rightSolenoid.set(ArmPosition.UP.value); 
        }
        SmartDashboard.putString("arm up?", "YES!");
        pistonsCurrentPosition = ArmPosition.UP;
    }

    /**
     * Release both arm pistons
     */
    public void lowerArm() {
        pistonsCurrentPosition = ArmPosition.DOWN;

        leftSolenoid.set(ArmPosition.DOWN.value);
        rightSolenoid.set(ArmPosition.DOWN.value);
        SmartDashboard.putString("arm up?", "NO!");
    }

    /**
     * 
     * @return Current arm position (up, down, neutral)
     */
    public ArmPosition getPistonsPosition() {
        return pistonsCurrentPosition;
    }

    public double getCurrentAngle() {
        return pistonsCurrentPosition.getAngle();
    }

    /**
     * Extend/retract telescope
     * 
     * @param position (0, 22)
     */
    public void setTelescopePosition(double position) {
        telescopeTargetPosition = position;
        // soft limits appear to be broken so use code limits for now
        if (position > MAX_POSITION) {
           telescopeTargetPosition = MAX_POSITION;
        } else if (position < MIN_POSITION) {
            telescopeTargetPosition = MIN_POSITION;
        }
        SmartDashboard.putNumber("arm target position", telescopeTargetPosition);
    }

    public double getTelescopePosition() {
        return telescopeController1.getEncoder().getPosition();
    }

    public void zeroTelescope() {
        telescopeController1.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        if (telescopeController1.getEncoder().getPosition() < 1 && telescopeTargetPosition < 1) {
            // near bottom, put tiny bit of motor to avoid unspooling string
            telescopeController1.set(0.003);
        } else if (telescopeTargetPosition <= 0 ) {
            // let arm gravity drop to transport position
            telescopeController1.set(0);
        } else {
            telescopeController1.getPIDController().setReference(telescopeTargetPosition, ControlType.kPosition);
        }

        SmartDashboard.putNumber("arm position", telescopeController1.getEncoder().getPosition());
    }
}
