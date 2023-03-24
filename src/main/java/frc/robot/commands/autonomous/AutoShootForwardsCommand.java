package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootScoreCommand;
import frc.robot.commands.instant.SetWristAngleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.TargetSelection.Height;

public class AutoShootForwardsCommand extends SequentialCommandGroup {
  
    private static final double CLAW_SPEED = 0.8;
    
    /** Creates a new AutoShootBackwardsCommand. */
    public AutoShootForwardsCommand(Arm arm, Wrist wrist, Claw claw) {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        // Raise arm and set wrist to correct angle
        new SetWristAngleCommand(wrist, Wrist.SCORE_CUBE_FORWARD),
        
        // Wait 1 second for arm + wrist to be in position
        new WaitCommand(1),
        
        // Spin the claw to score high cube
        new ShootScoreCommand(Height.HIGH , claw),
        
        // Set arm and wrist to transport position
        new SetWristAngleCommand(wrist, Wrist.TRANSPORT)
      );
    }
}
