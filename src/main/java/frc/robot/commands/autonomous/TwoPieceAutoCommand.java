package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.pathplannerUtils.CreatePathUtils;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class TwoPieceAutoCommand extends SequentialCommandGroup {
    public TwoPieceAutoCommand(CreatePathUtils pathUtils, Gyro gyro, Drivetrain drivetrain) {
        addCommands(
            pathUtils.createPathCommand("test 1", 1, 1, true),
            new TurnToAngleCommand(gyro, drivetrain, 180),
            pathUtils.createPathCommand("test 2", 1, 1)
        );
    }
}
