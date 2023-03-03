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
    private static final double VOLTS_TO_DEGREES_SLOPE = 67.95;
    private static final double VOLTS_TO_DEGREES_CONSTANT = -82.16;

    // Voltage required to hold arm up at horizontal
    // 0.075 is the experimentally determined motor percentage that does that
    private static final double GRAVITY_COMPENSATION = 0.075 * 12;

    // Wrist angle
    private double angle = 0;

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
        angle = getAngle();
    }

    /**
     * 
     * @param position -1 to 1
     */
    public void goToPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }

    public double getAngle() {
        double potVoltage = pot.getPosition();
        return potVoltage * VOLTS_TO_DEGREES_SLOPE + VOLTS_TO_DEGREES_CONSTANT + ARM_ANGLE;
    }

    public double convertAngleToVolts(double angle) {
        return angle * (1 / VOLTS_TO_DEGREES_SLOPE) - VOLTS_TO_DEGREES_CONSTANT * (1 / VOLTS_TO_DEGREES_SLOPE);
    }

    /**
     * 
     * @param angle degrees
     */
    public void setAngle(double angle) { // abc1239+10=21 road work ahead, i sure hope it does. David was here.......
        this.angle = angle;
    }

    private double getArbitraryFeedforward() {
        // Gravity compensation constant * cos(current angle)
        return GRAVITY_COMPENSATION * Math.cos(Math.toRadians(getAngle()));
    }

    @Override
    public void periodic() {
        // Wrist PID is always being set to the given angle. Wrist is never floppy
        pidController.setReference(convertAngleToVolts(angle), ControlType.kPosition, 0, getArbitraryFeedforward());

        SmartDashboard.putNumber("wrist target angle", angle);
        SmartDashboard.putNumber("wrist target volts", convertAngleToVolts(angle));
        SmartDashboard.putNumber("wrist AFF", getArbitraryFeedforward());

        SmartDashboard.putNumber("wrist current angle", getAngle());
        SmartDashboard.putNumber("wrist current volts", pot.getPosition());

    }
}
