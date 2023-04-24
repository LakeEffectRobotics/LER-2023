package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

    public final CANSparkMax leftLeadController;
    public final CANSparkMax rightLeadController;

    // Wheel size 6.25"
    // Gearing
    // Stage 1 - 14:50
    // Stage 2 - 18:46
    private static final double METERS_PER_REV = 0.15875 * Math.PI * (14 / 50.0) * (18 / 46.0);

    private final DifferentialDriveOdometry odometry;
    public final Field2d field = new Field2d();

    // Temporary gyro
    // final ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    Gyro gyro = new Gyro();

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    // kV from characterization tool , divide by 12 to get percentage
    private static final double kF = 2.3589 / 12;
    // PID
  //  private static final double kP = 0.5914;
    private static final double kP = 0.096;
    private static final double kI = 0;
    private static final double kD = 0;

    private final PIDController pid2 = new PIDController(2, 0, 0);

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.16534, 2.3589, 0.47591);

    // Max speed in m/s
    public final double MAX_SPEED = 4.5;
    public double speedMultiplier = 1;

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

     //   SmartDashboard.putData("Field View", field);

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

    /**
     * 
     * @param leftSpeed  left wheel velocity (m/s)
     * @param rightSpeed right wheel velocity (m/s)
     */
    public void velocityTankDrive(double leftSpeed, double rightSpeed) {
        leftLeadController.setVoltage(ff.calculate(leftSpeed) + pid2.calculate(leftEncoder.getVelocity(), leftSpeed));
        rightLeadController.setVoltage(ff.calculate(rightSpeed) + pid2.calculate(rightEncoder.getVelocity(), rightSpeed));

        //leftLeadController.getPIDController().setReference(leftSpeed, ControlType.kVelocity);
        //rightLeadController.getPIDController().setReference(rightSpeed, ControlType.kVelocity);
    }

    // Percent tank drive for regular joystik driving
    public void tankDrive(double left, double right) {
        leftLeadController.set(left);
        rightLeadController.set(right);
    }

    /**
     * Set speed multiplier to slow down driving on velocity drive
     * @param multiplier [0, 1]
     */
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }
    
    // Arcade drive
    /**
     * @param speed    linear velocity (m/s)
     * @param rotation angular velocity (rad/s)
     */
    public void arcadeDrive(double speed, double rotation) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));
        tankDrive(wheelSpeeds.leftMetersPerSecond / MAX_SPEED, wheelSpeeds.rightMetersPerSecond / MAX_SPEED);
    }

    public void stop() {
        leftLeadController.set(0);
        rightLeadController.set(0);
    }

    // Update the odometry every 20ms
    @Override
    public void periodic() {
     //   field.setRobotPose(getPose());
      //  SmartDashboard.putNumber("Right velocity", rightEncoder.getVelocity());
       // SmartDashboard.putNumber("Left velocity", leftEncoder.getVelocity());
        
        odometry.update(
                gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

        // Extra diagnositcs, breaks best practices alot but I need the data
        // mfw we kill 1/3 of the left drive motors and dont realize until 2 comps later :facepalm:
        //SmartDashboard.putNumber("Left 1 Applied", RobotMap.leftController1.getAppliedOutput());
        //SmartDashboard.putNumber("Left 2 Applied", RobotMap.leftController2.getAppliedOutput());
        //SmartDashboard.putNumber("Left 3 Applied", RobotMap.leftController3.getAppliedOutput());
        //SmartDashboard.putNumber("Right 1 Applied", RobotMap.rightController1.getAppliedOutput());
        //SmartDashboard.putNumber("Right 2 Applied", RobotMap.rightController2.getAppliedOutput());
        //SmartDashboard.putNumber("Right 3 Applied", RobotMap.rightController3.getAppliedOutput());
        //
        //SmartDashboard.putNumber("Left 1 Bus", RobotMap.leftController1.getBusVoltage());
        //SmartDashboard.putNumber("Left 2 Bus", RobotMap.leftController2.getBusVoltage());
        //SmartDashboard.putNumber("Left 3 Bus", RobotMap.leftController3.getBusVoltage());
        //SmartDashboard.putNumber("Right 1 Bus", RobotMap.rightController1.getBusVoltage());
        //SmartDashboard.putNumber("Right 2 Bus", RobotMap.rightController2.getBusVoltage());
        //SmartDashboard.putNumber("Right 3 Bus", RobotMap.rightController3.getBusVoltage());
        //
        //SmartDashboard.putNumber("Left 1 Current", RobotMap.leftController1.getOutputCurrent());
        //SmartDashboard.putNumber("Left 2 Current", RobotMap.leftController2.getOutputCurrent());
        //SmartDashboard.putNumber("Left 3 Current", RobotMap.leftController3.getOutputCurrent());
        //SmartDashboard.putNumber("Right 1 Current", RobotMap.rightController1.getOutputCurrent());
        //SmartDashboard.putNumber("Right 2 Current", RobotMap.rightController2.getOutputCurrent());
        //SmartDashboard.putNumber("Right 3 Current", RobotMap.rightController3.getOutputCurrent());
    }
}
