package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.SpinClawCommand;
import frc.robot.commands.SpinClawCommand.Direction;
import frc.robot.commands.instant.LowerArmCommand;
import frc.robot.commands.instant.RaiseArmCommand;
import frc.robot.commands.instant.SetWristAngleCommand;
import frc.robot.commands.instant.TransportPositionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.TargetSelection.Type;


public class AutoMidCubeBackwards extends SequentialCommandGroup {
    private static final double CLAW_SPEED = 0.65;

    /**
     * @param arm
     * @param wrist
     */
    public AutoMidCubeBackwards(Arm arm, Wrist wrist, Claw claw) {

        addCommands(
            // Raise arm and set wrist to correct angle
            //new RaiseArmCommand(arm, true),
            new LowerArmCommand(arm),
      
            new SetWristAngleCommand(wrist, Wrist.TRANSPORT),
            
            // Wait 1 second for arm + wrist to be in position
            new WaitUntilCommand(() -> Math.abs(wrist.getCurrentAngle()) - Wrist.TRANSPORT < 10)
            .withTimeout(1),
            
            // Spin the claw for 1 second at 100% power to shoot the cube
            new SpinClawCommand(claw, Direction.OUT, () -> CLAW_SPEED, Type.CUBE).withTimeout(0.2),
            
            // Set arm and wrist to transport position
            new TransportPositionCommand(arm, wrist)
            );
    }
}
