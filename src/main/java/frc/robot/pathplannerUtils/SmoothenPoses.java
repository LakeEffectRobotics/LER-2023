package frc.robot.pathplannerUtils;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SmoothenPoses {
    Pose2d smoothedPose;
    HashMap<Double, Pose2d> observedPoses;
    double maxDistanceBetween = 1;

    public boolean tryAddPose(Pose2d newPose) {
        double distanceBetween = newPose.getTranslation().getDistance(smoothedPose.getTranslation());

        // if new pose is impossibly far from current avg pose, reject it
        if (distanceBetween > maxDistanceBetween) {
            return false;
        } else {
            smoothenPoses(newPose);
            return true;
        }

        // otherwise, add it to the pose list and smoothen it
    }

    private void smoothenPoses(Pose2d newPose) {
        // average poses
        double x = 0;
        double y = 0;
        double sin = 0;
        double cos = 0;
        
        int sampleSize = 0;

        for (Pose2d pose : observedPoses.values()) {
            sampleSize = sampleSize + 1;
            
            x += pose.getTranslation().getX();
            y += pose.getTranslation().getY();
            cos += pose.getRotation().getCos();
            sin += pose.getRotation().getSin();
        }

        x /= sampleSize;
        y /= sampleSize;
        sin /= sampleSize;
        cos /= sampleSize;
    }
}
