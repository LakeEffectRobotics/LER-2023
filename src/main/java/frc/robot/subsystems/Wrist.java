package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    CANSparkMax wristController;

    public SparkMaxLimitSwitch forwardLimit;
    public SparkMaxLimitSwitch reverseLimit;

    private SparkMaxAnalogSensor pot;

    private SparkMaxPIDController pidController;

    private static final double kF = 0;
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double MAX_OUTPUT = -1;
    private static final double MIN_OUTPUT = -1;

    // Angle of the arm relative to horizontal ground (degrees)
    // Currently a constant as arm prototype is stationary
    private static final double ARM_ANGLE = 1.6;

    // Function to convert from potentiometer volts to arm degrees above horizontal
    private static final double VOLTS_TO_DEGREES_SLOPE = 97.73;
    private static final double VOLTS_TO_DEGREES_CONSTANT = 113.7;

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
}
