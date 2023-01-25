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
}
