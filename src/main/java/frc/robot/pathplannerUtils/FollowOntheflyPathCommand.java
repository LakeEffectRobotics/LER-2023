package frc.robot.pathplannerUtils;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command used by CreatePathUtils to wrap path following for paths initialized
 * at runtime, since otherwise they are created with robot's starting pose
 * 
 * This should not be used directly anywhere else, so is package-level access
 */
class FollowOntheflyPathCommand extends CommandBase {
    private Supplier<Pose2d> currentPose;
    private Pose2d targetPose;
    private double maxVelocity;
    private double maxAcceleration;
    private RamseteAutoBuilder autoBuilder;
    private Command autoFollowPathCommand;

    public FollowOntheflyPathCommand(Supplier<Pose2d> currentPose, Pose2d targetPose, double maxVelocity,
            double maxAcceleration, RamseteAutoBuilder autoBuilder) {
        this.currentPose = currentPose;
        this.targetPose = targetPose;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.autoBuilder = autoBuilder;
    }

    @Override
    public void initialize() {
        PathPlannerTrajectory path = PathPlanner.loadPath("drivearound",
                new PathConstraints(maxVelocity, maxAcceleration));
        PathPlannerTrajectory onTheFlyPath = PathPlanner.generatePath(new PathConstraints(maxVelocity, maxAcceleration),
                // new PathPoint(currentPose.get().getTranslation(),
                // currentPose.get().getRotation()),
                // temporary: try manual starting location test jitter
                new PathPoint(new Translation2d(12.7, 6.4), new Rotation2d(0)),
                new PathPoint(targetPose.getTranslation(),
                        targetPose.getRotation()));

        // Build and return path command
        autoFollowPathCommand = autoBuilder.fullAuto(path);

        autoFollowPathCommand.initialize();

        SmartDashboard.putString("fly current pose", currentPose.get().toString());
        SmartDashboard.putString("fly target pose", targetPose.toString());

    }

    @Override
    public void execute() {
        autoFollowPathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        autoFollowPathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return autoFollowPathCommand.isFinished();
    }

}