package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

    CANSparkMax leftLeadController;
    CANSparkMax rightLeadController;

    // Temporary fake gyro
    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private static RelativeEncoder leftEncoder = RobotMap.leftController1.getEncoder();
    private static RelativeEncoder rightEncoder = RobotMap.rightController1.getEncoder();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.19);

    public Drivetrain(CANSparkMax leftLeadController, CANSparkMax rightLeadController) {
        this.leftLeadController = leftLeadController;
        this.rightLeadController = rightLeadController;
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
