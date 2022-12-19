package org.team5557.vision;

import org.team5557.Constants;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.apriltag.AprilTag;
import edu.wpi.first.wpilibj.apriltag.AprilTagFieldLayout;

public class Hotspot {
    private final Pose2d goalPosition;
    private final double maxPathLength;
    private final Pose2d errorRadius;
    private final AprilTag associatedVisionTarget;
    private final PathConstraints pathConstraints;
    private final boolean shouldLock;

    public Hotspot(Pose2d goalPosition, AprilTag associatedVisionTarget, PathConstraints pathConstraints, boolean shouldLock) {
        this.goalPosition = goalPosition;
        this.maxPathLength = Constants.copilot.max_path_length;
        this.errorRadius = Constants.copilot.error_radius;
        this.associatedVisionTarget = associatedVisionTarget;
        this.pathConstraints = pathConstraints;
        this.shouldLock = shouldLock;
    }

    public Pose2d getGoalPosition() {
        return goalPosition;
    }

    public AprilTag getAssociatedTarget() {
        return associatedVisionTarget;
    }
    
    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    public double getMaxPathLength() {
        return maxPathLength;
    }

    public Pose2d getErrorRadius() {
        return errorRadius;
    }

    public boolean getShouldX() {
        return shouldLock;
    }
}
