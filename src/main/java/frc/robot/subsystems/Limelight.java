package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable table;

  public Limelight() {

  }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    SmartDashboard.putNumber("limelightX", getX());
    SmartDashboard.putNumber("LimelightY", getY());
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

  public void setPipline(int pipeline) {
    table.getEntry("pipeline").setInteger(pipeline);
  }

  public void getPose() {
    double[] pose = table.getEntry("botpose").getDoubleArray(new double[6]);
    System.out.println(pose[1] + " " + pose[2] + " " + pose[3] + " " + pose[4]);
  }
}
