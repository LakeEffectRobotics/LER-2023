package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable table;

    public enum Pipeline {
        LIMELIGHT(0),
        APRILTAG(1),
        CUBE(3),
        CONE(4);

       private final double pipelineId;

        Pipeline(double pipelineId) {
            this.pipelineId = pipelineId;
        }
    }

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setPipeline(Pipeline.APRILTAG);
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
 * set limelight pipeline 
 * @param pipeline enum of pipeline
 */
  public void setPipeline(Pipeline pipeline) {
    table.getEntry("pipeline").setDouble(pipeline.pipelineId);
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
