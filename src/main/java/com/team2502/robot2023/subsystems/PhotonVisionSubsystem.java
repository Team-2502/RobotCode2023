package com.team2502.robot2023.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;

import com.team2502.demo2022.Constants.Subsystems.PhotonVision;

public class PhotonVisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    AprilTagFieldLayout fieldLayout;
    
    public PhotonVisionSubsystem() {
        camera = new PhotonCamera(PhotonVision.CAMERA_NAME);
        try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory().getAbsolutePath()+"/2022-rapidreact.json");
		} catch (IOException e) {
			e.printStackTrace();
		}
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);


    }

    @Override
    public void periodic() {
        //PhotonPipelineResult result = camera.getLatestResult();
        //result.getBestTarget().get
        //SmartDashboard.putNumber("PHOTONVAL", value)
    }
}
