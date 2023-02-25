package com.team2502.robot2023.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.Optional;

import com.team2502.robot2023.Constants.Subsystems.PhotonVision;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.AprilTags;

public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;
    private Pose2d latestPose;
    private boolean newPoseThisFrame;
    
    public PhotonVisionSubsystem() {
        camera = new PhotonCamera(PhotonVision.CAMERA_NAME);
        fieldLayout = AprilTags.field;
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.Subsystems.PhotonVision.ROBOT_TO_PHOTONVISION);

        latestPose = new Pose2d();
        newPoseThisFrame = false;
    }

    @Override
    public void periodic() {
        /*
        Optional<EstimatedRobotPose> pose = estimator.update();
        
        newPoseThisFrame = pose.isPresent();
        if (pose.isPresent()) {
            latestPose = pose.get().estimatedPose.toPose2d();
        }

        SmartDashboard.putNumber("PV Posex", getPose().getX());
        SmartDashboard.putNumber("PV Posey", getPose().getY());
        SmartDashboard.putNumber("PV Poser", getPose().getRotation().getDegrees());
        */
    }

    public boolean newPoseThisFrame() {
        return newPoseThisFrame;
    }

    public Pose2d getPose() {
        return latestPose;
    }
}
