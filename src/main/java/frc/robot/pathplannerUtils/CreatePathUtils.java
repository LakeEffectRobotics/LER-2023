package frc.robot.pathplannerUtils;

import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ApriltagPoseCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class CreatePathUtils {
    Drivetrain drivetrain;
    Limelight limelight;

    // Create autobuilder which will build any path
    RamseteAutoBuilder autoBuilder;

    public static final HashMap<String, Command> eventMap = new HashMap<>();

    public CreatePathUtils(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetPose,
                new RamseteController(), drivetrain.kinematics, drivetrain::velocityTankDrive, eventMap, drivetrain);

        // Global event map
        // Add all the predefined events used by paths to the global event map
        eventMap.put("apriltagpose", new ApriltagPoseCommand(limelight, drivetrain));
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
        PathPlannerTrajectory path = PathPlanner.loadPath("drivearound",
                new PathConstraints(maxVelocity, maxAcceleration));

        // Build and return path command
        // Command autoFollowPathCommand = autoBuilder.fullAuto(path);

        // try using pathplanner's ramsetecommand
        Command autoFollowPathCommand = new PPRamseteCommand(path, drivetrain::getPose, new RamseteController(),
                drivetrain.kinematics, drivetrain::velocityTankDrive, drivetrain);

        return autoFollowPathCommand;
    }

    /**
     * Create follow path command to move from current bot pose to a target pose
     * * @return
     */
    public Command createOntheflyPath(Supplier<Pose2d> currentPose, Pose2d targetPose, double maxVelocity,
            double maxAcceleration) {
        return new FollowOntheflyPathCommand(currentPose, targetPose, maxVelocity, maxAcceleration, autoBuilder);
    }
}
