package com.team2502.robot2023.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import com.team2502.robot2023.Constants.Subsystems.Field;

public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonCamera cameraIntake;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;
    private Pose2d latestPose;
    private boolean newPoseThisFrame;
    private DrivetrainSubsystem drivetrain;


    public Optional<PhotonTrackedTarget> gamePiece;
    
    public PhotonVisionSubsystem(DrivetrainSubsystem drivetrain) {
        camera = new PhotonCamera(PhotonVision.CAMERA_NAME);
        fieldLayout = Field.apriltagPositions;
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.Subsystems.PhotonVision.ROBOT_TO_PHOTONVISION);


        cameraIntake = new PhotonCamera("intake");

        latestPose = new Pose2d();
        newPoseThisFrame = false;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // apriltags
        estimator.setReferencePose(drivetrain.getPose());
        Optional<EstimatedRobotPose> pose = estimator.update();
        
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        if (target != null) {
            SmartDashboard.putNumber("pv ambi", target.getPoseAmbiguity());
            SmartDashboard.putNumber("pv area", target.getArea());
        }

        newPoseThisFrame = pose.isPresent() && target != null && target.getArea() > 1.5;
        if (pose.isPresent() && target != null && target.getArea() > 1.5) {
            latestPose = pose.get().estimatedPose.toPose2d();
        }

        // intake
        if (cameraIntake.hasTargets()) {
            target = cameraIntake.getLatestResult().getBestTarget();

            if (target==null) { return; } // horrible api

            gamePiece = Optional.of(target);

            SmartDashboard.putNumber("PV item x", target.getYaw());
            SmartDashboard.putNumber("PV item y", target.getPitch());
        }

        SmartDashboard.putNumber("PV Posex", getPose().getX());
        SmartDashboard.putNumber("PV Posey", getPose().getY());
        SmartDashboard.putNumber("PV Poser", getPose().getRotation().getDegrees());
    }

    public boolean newPoseThisFrame() {
        return newPoseThisFrame;
    }

    public Pose2d getPose() {
        return latestPose;
    }
}
