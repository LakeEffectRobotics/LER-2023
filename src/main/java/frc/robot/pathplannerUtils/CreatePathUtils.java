package frc.robot.pathplannerUtils;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CreatePathUtils {
    Drivetrain drivetrain;

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
    public Command CreatePathCommand(String pathName, double maxVelocity, double maxAcceleration) {

        // Load the path from the .path file created by pathplanner
        PathPlannerTrajectory path = PathPlanner.loadPath("drivearound",
                new PathConstraints(maxVelocity, maxAcceleration));

        // Todo: event map
        HashMap<String, Command> eventMap = new HashMap<>();

        // Create autobuilder which will build the path
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetPose,
                new RamseteController(), drivetrain.kinematics, drivetrain::velocityTankDrive, eventMap, drivetrain);

        // Build path command
        Command autoFollowPathCommand = autoBuilder.fullAuto(path);

        return autoFollowPathCommand;
    }
}
