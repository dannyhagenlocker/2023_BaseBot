package org.team5557.Vision;

import java.sql.Time;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team5557.Constants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.apriltag.AprilTag;
import edu.wpi.first.wpilibj.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase {
    public static final AprilTag zero_zero = new AprilTag(0, new Pose3d());
    public static final AprilTag five_zero = new AprilTag(1, new Pose3d(5,0,0.3, new Rotation3d(0, 0, Units.degreesToRadians(180.0))));
    public static final AprilTag five_five = new AprilTag(2, new Pose3d(5,5,0.3, new Rotation3d(0, 0, Units.degreesToRadians(180.0))));
    public static final List<AprilTag> target_poses = Collections.unmodifiableList(
        List.of(
            zero_zero,
            five_zero,
            five_five
        )
    );
    AprilTagFieldLayout tag_layout = new AprilTagFieldLayout(target_poses, 10, 10);

    private double statusExpiryTime = 0.0;
    private final Swerve swerve;
    private final RobotContainer container;
    private final PhotonCameraExtension photonCamera;
    private final List<PhotonCameraExtension> camera_list;

    public VisionManager(RobotContainer container) {
        this.container = container;
        this.swerve = container.getSwerve();
        this.photonCamera = new PhotonCameraExtension("limelight", new Transform3d());
        camera_list = Collections.unmodifiableList(
            List.of(
                photonCamera
            )
        );

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.vision_readout_key);
        tab.addCamera("Limelight", "Limelight", "http://limelight.local:5800", "http://10.29.10.11:5800")
            .withSize(3, 3)
            .withPosition(4, 0);
    }

    @Override
    public void periodic() {
        for (PhotonCameraExtension camera : camera_list) {
            var pipelineResult = camera.getLatestResult();
            if (!pipelineResult.equals(camera.getLastPipelineResult()) && pipelineResult.hasTargets()) {
              camera.setLastPipelineResult(pipelineResult);
              //double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);
              double imageCaptureTime = pipelineResult.getTimestampSeconds();
              var target = pipelineResult.getBestTarget();
              var fiducialId = target.getFiducialId();
              var targetPose = tag_layout.getTagPose(fiducialId);
              if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && targetPose.isPresent()) {
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.get().transformBy(camToTarget.inverse());
                var visionMeasurement = camPose.transformBy(camera.CAMERA_TO_ROBOT);
                Hotspot hotspot = container.getGoalPlanner().getHotspot();
                if(fiducialId == hotspot.getAssociatedTarget().ID && visionMeasurement.toPose2d().getTranslation().getNorm() < Constants.estimator.max_high_accuracy_distance) {
                    swerve.getEstimator().addVisionMeasurement(visionMeasurement.toPose2d(), imageCaptureTime, Constants.estimator.highAccuracyVisionStdDevs);
                    statusExpiryTime = Timer.getFPGATimestamp() + 0.1;
                } else {
                    swerve.getEstimator().addVisionMeasurement(visionMeasurement.toPose2d(), imageCaptureTime, Constants.estimator.normalVisionStdDevs);
                }
              }
            }
        }
    }

    public boolean associatedTargetVisible() {
        return statusExpiryTime >= Timer.getFPGATimestamp();
    }
}
