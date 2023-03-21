package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable table;

  private static final int LIMELIGHT_PIPELINE = 0;
  private static final int APRILTAG_PIPELINE = 1;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    useApriltagPipeline();
  }

  @Override
  public void periodic() {
  }

  public double getX() {
    return (table.getEntry("tx").getDouble(0.0));
  }

  public double getY() {
    return (table.getEntry("ty").getDouble(0.0));
  }

  public double getTargetArea() {
    return (table.getEntry("ta").getDouble(0.0));
  }

  public double getSkew() {
    return (table.getEntry("ts").getDouble(0.0));
  }

  public boolean isThereValidTarget() {
    // tv: Whether the limelight has any valid targets (0 or 1)
    double tv = (table.getEntry("tv").getInteger(0));
    if (tv == 0) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Make limelight look for apriltags
   * 
   */
  public void useApriltagPipeline() {
    table.getEntry("pipeline").setDouble(APRILTAG_PIPELINE);
  }

  /**
   * Make limelight look for retroreflective
   * 
   */
  public void useLimelightPipeline() {
    table.getEntry("pipeline").setDouble(LIMELIGHT_PIPELINE);
  }

  /**
   * 
   * @return current apriltag-determined pose, if there are any apriltags in view
   */
  public Pose2d getPose() {
    if (isThereValidTarget()) {
      double[] pose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
      return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
    } else {
      return null;
    }

  }

}
