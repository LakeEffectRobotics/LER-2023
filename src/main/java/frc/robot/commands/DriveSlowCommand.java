package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSlowCommand extends CommandBase {
    private final Drivetrain drivetrain;

    public DriveSlowCommand(Drivetrain drivetrain) {
      this.drivetrain = drivetrain;
    }
  
    @Override
    public void initialize() {
      drivetrain.setSpeedMultiplier(0.3);
    }
  
    @Override
    public void end(boolean interrupted) {
      drivetrain.setSpeedMultiplier(1);
    }
}
