package frc.robot.pathplannerUtils;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ApriltagPoseCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.autonomous.AutoBalanceCommand;
import frc.robot.commands.autonomous.AutoBumpBackCommand;
import frc.robot.commands.autonomous.AutoIntakeCommand;
import frc.robot.commands.autonomous.AutoShootBackwardsCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Wrist;

public class CreatePathUtils {
    Drivetrain drivetrain;
    Limelight limelight;
    Arm arm;
    Wrist wrist;
    Claw claw;
    Gyro gyro;
    TargetSelection targetSelection;

    // Create autobuilder which will build any path
    RamseteAutoBuilder autoBuilder;

    public static final HashMap<String, Command> eventMap = new HashMap<>();

    public CreatePathUtils(Drivetrain drivetrain, Limelight limelight, Arm arm, Wrist wrist, Claw claw, Gyro gyro, TargetSelection targetSelection) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.gyro = gyro;
        this.targetSelection = targetSelection;

        autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetPose,
               new RamseteController(), drivetrain.kinematics, drivetrain::velocityTankDrive, eventMap, true, drivetrain);

        // Global event map
        // Add all the predefined events used by paths to the global event map
        eventMap.put("apriltagpose", new ApriltagPoseCommand(limelight, drivetrain));
        
        eventMap.put("intake cube", new AutoIntakeCommand(drivetrain, arm, wrist, claw, targetSelection));
        eventMap.put("outtake cube", new AutoShootBackwardsCommand(arm, wrist, claw, targetSelection));

        eventMap.put("balance reversed", new AutoBalanceCommand(gyro, drivetrain));
        eventMap.put("balance forward", new AutoBalanceCommand(gyro, drivetrain));

        eventMap.put("turn right", (new TurnToAngleCommand(gyro, drivetrain, 0)).withTimeout(0.7));
        eventMap.put("bump back", new AutoBumpBackCommand(70, drivetrain));
    }

    /**
     * 
     * @param pathName        Name of the path file (without the ".path")
     * @param maxVelocity     (m/s)
     * @param maxAcceleration (m/s)
     * @return command that automatically follows the path
     */
    public Command createPathCommand(String pathName, double maxVelocity, double maxAcceleration) {

        // Load the path from the .path file created by pathplanner
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, new PathConstraints(maxVelocity, maxAcceleration));

        // Build and return path command
        Command autoFollowPathCommand = autoBuilder.fullAuto(path);
        return autoFollowPathCommand;
    }

    /**
     * Create follow path command to move from current bot pose to a target pose
     * * @return
     */
    public Command createOntheflyPath(Pose2d currentPose, Pose2d targetPose, double maxVelocity,
            double maxAcceleration) {
        // Get current bot pose
        // Pose2d currentPose = drivetrain.getPose();

        // Create new path to move from current pose to the given targetPose
        PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(maxVelocity, maxAcceleration),
                new PathPoint(currentPose.getTranslation(), currentPose.getRotation()),
                new PathPoint(targetPose.getTranslation(),
                        targetPose.getRotation()));

        // Build and return path command
        Command autoFollowPathCommand = autoBuilder.fullAuto(path);
        return autoFollowPathCommand;
    }
}
