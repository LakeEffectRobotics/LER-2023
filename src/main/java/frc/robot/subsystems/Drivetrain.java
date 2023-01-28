package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

    CANSparkMax leftLeadController;
    CANSparkMax rightLeadController;

    private final DifferentialDriveOdometry m_odometry;
    // Temporary fake gyro
    final AnalogGyro gyro = new AnalogGyro(0);

    // Temporary fake gyro
    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private static RelativeEncoder leftEncoder = RobotMap.leftController1.getEncoder();
    private static RelativeEncoder rightEncoder = RobotMap.rightController1.getEncoder();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.19);

    public Drivetrain(CANSparkMax leftLeadController, CANSparkMax rightLeadController) {
        this.leftLeadController = leftLeadController;
        this.rightLeadController = rightLeadController;

        m_odometry = new DifferentialDriveOdometry(
                gyro.getRotation2d(),
                this.getLeftEncoder().getPosition(), this.getRightEncoder().getPosition());

    }

    public void updateOdometry() {
        m_odometry.update(
                gyro.getRotation2d(), this.getLeftEncoder().getPosition(), this.getRightEncoder().getPosition());
    }

    public RelativeEncoder getLeftEncoder() {
        return (RobotMap.leftController1.getEncoder());
    }

    public RelativeEncoder getRightEncoder() {
        return (RobotMap.rightController1.getEncoder());
    }

    public void resetEncoderPosition() {
        RobotMap.leftController1.getEncoder().setPosition(0);
        RobotMap.leftController2.getEncoder().setPosition(0);
        RobotMap.leftController3.getEncoder().setPosition(0);
        RobotMap.rightController1.getEncoder().setPosition(0);
        RobotMap.rightController2.getEncoder().setPosition(0);
        RobotMap.rightController3.getEncoder().setPosition(0);
    }

    public void setSpeedOutput(DifferentialDriveWheelSpeeds wheelSpeeds) {
        final double leftOutput = wheelSpeeds.leftMetersPerSecond;
        final double rightOutput = wheelSpeeds.rightMetersPerSecond;
        // todo? add pid stuff ?

        setOutput(leftOutput, rightOutput);
    }

    /**
     * @param speed    linear velocity (m/s)
     * @param rotation angular velocity (rad/s)
     */
    public void drive(double speed, double rotation) {
        setSpeedOutput(kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation)));
    }

    public void setOutput(double left, double right) {
        // Other controllers are followers
        leftLeadController.set(left);
        rightLeadController.set(right);
    }

    public void stop() {
        leftLeadController.set(0);
        rightLeadController.set(0);
    }
}
