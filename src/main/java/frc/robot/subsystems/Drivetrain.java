package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    public final CANSparkMax leftLeadController;
    public final CANSparkMax rightLeadController;

    // Wheel size 6.25"
    // Gearing
    // Stage 1 - 14:50
    // Stage 2 - 18:46
    private static final double METERS_PER_REV = 0.15875 * Math.PI * (14 / 50.0) * (18 / 46.0);

    private final DifferentialDriveOdometry odometry;
    private final Field2d field = new Field2d();

    // Temporary gyro
    // final ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    // kV from characterization tool -> units of meters
    private static final double kF = 1.24 * METERS_PER_REV;
    // PID
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;

    // Feedforward
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    // Max speed in m/s
    public static final double MAX_SPEED = 4;

    // Robot track width 19"
    public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(19));

    // Create new Drivetrain
    public Drivetrain(CANSparkMax leftLeadController, CANSparkMax rightLeadController) {
        this.leftLeadController = leftLeadController;
        this.rightLeadController = rightLeadController;

        this.leftEncoder = leftLeadController.getEncoder();
        this.rightEncoder = rightLeadController.getEncoder();

        resetEncoderPosition();

        odometry = new DifferentialDriveOdometry(
                gyro.getRotation2d(),
                leftEncoder.getPosition(), rightEncoder.getPosition());

        // Setup unit conversions to get meters and meters/second
        leftLeadController.getEncoder().setPositionConversionFactor(METERS_PER_REV);
        // MUST CONVERT FROM MINUTES to seconds
        leftLeadController.getEncoder().setVelocityConversionFactor(METERS_PER_REV * 1 / 60.0);
        rightLeadController.getEncoder().setPositionConversionFactor(METERS_PER_REV);
        rightLeadController.getEncoder().setVelocityConversionFactor(METERS_PER_REV * 1 / 60.0);

        leftLeadController.getPIDController().setFF(kF);
        leftLeadController.getPIDController().setD(kD);
        leftLeadController.getPIDController().setI(kI);
        leftLeadController.getPIDController().setP(kP);

        rightLeadController.getPIDController().setFF(kF);
        rightLeadController.getPIDController().setD(kD);
        rightLeadController.getPIDController().setI(kI);
        rightLeadController.getPIDController().setP(kP);

        SmartDashboard.putData("Field View", field);

        resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        gyro.reset();
    }

    // Odometry methods
    public void updateOdometry() {
        odometry.update(
                gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public void resetEncoderPosition() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {

        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        resetEncoderPosition();
        odometry.resetPosition(
                gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
    }

    // Arcade drive
    /**
     * @param speed    linear velocity (m/s)
     * @param rotation angular velocity (rad/s)
     */
    public void arcadeDrive(double speed, double rotation) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));
        velocityTankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * 
     * @param leftSpeed  left wheel velocity (m/s)
     * @param rightSpeed right wheel velocity (m/s)
     */
    public void velocityTankDrive(double leftSpeed, double rightSpeed) {
        // Other controllers are followers
        double leftOutput = leftSpeed * MAX_SPEED;
        double rightOutput = rightSpeed * MAX_SPEED;
        SmartDashboard.putNumber("LEFT TARGET VELOCITY", leftOutput);
        SmartDashboard.putNumber("right target VELOCITY", rightOutput);

        leftLeadController.getPIDController().setReference(leftOutput, ControlType.kVelocity);
        rightLeadController.getPIDController().setReference(rightOutput, ControlType.kVelocity);
    }

    public void setSpeeds(double leftSpeed, double rightSpeed) {
        final double leftFF = m_feedforward.calculate(leftSpeed);
        final double rightFF = m_feedforward.calculate(rightSpeed);
        leftLeadController.setVoltage(leftFF);
        rightLeadController.setVoltage(rightFF);
    }

    // Percent tank drive for regular joystik driving
    public void tankDrive(double left, double right) {
        leftLeadController.set(left);
        rightLeadController.set(right);
    }

    public void stop() {
        leftLeadController.set(0);
        rightLeadController.set(0);
    }

    // Update the odometry every 20ms
    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Right velocity", rightEncoder.getVelocity());
        SmartDashboard.putNumber("Left velocity", leftEncoder.getVelocity());

        SmartDashboard.putNumber("left encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("right encoder", rightEncoder.getPosition());
        odometry.update(
                gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
}
