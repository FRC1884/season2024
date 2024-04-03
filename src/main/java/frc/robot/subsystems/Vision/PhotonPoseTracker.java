package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotMap.VisionConfig;

public class PhotonPoseTracker {

    public PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera photonCamera;
    private boolean hasUpdated;
    private PhotonPipelineResult photonPipelineResult;
    private Pose2d estimatedVisionBotPose;
    private double visionEstimateTimestamp;
    private double distanceToBestTarget;
  
    private VisionConfig.CAMERA_TYPE cameraType;
    private boolean isMultiTag;
  
    public PhotonPoseTracker(PhotonPoseEstimator photonPoseEstimator, PhotonCamera photonCamera, VisionConfig.CAMERA_TYPE cameraType) {
      this.photonPoseEstimator = photonPoseEstimator;
      this.hasUpdated = false;
      this.photonCamera = photonCamera;
      estimatedVisionBotPose = new Pose2d();
      this.cameraType = cameraType;
      isMultiTag = false;
    }
  
    public void updateCameraPipelineResult(){
      photonPipelineResult = photonCamera.getLatestResult();
      visionEstimateTimestamp = photonPipelineResult.getTimestampSeconds();
    }
  
    public PhotonPipelineResult getPhotonPipelineResult(){
      return photonPipelineResult;
    }
  
    public void setUpdatedStatus(boolean updateStatus){
      hasUpdated = updateStatus; 
    }
  
    public double getCurrentTimestamp(){
      return visionEstimateTimestamp;
    }
  
    public void updateEstimatedBotPose(Pose2d currentVisionPose, boolean isMultiTag){
      estimatedVisionBotPose = currentVisionPose;
      hasUpdated = true;
      this.isMultiTag = isMultiTag;
    }
  
    public boolean hasUpdatedVisionEstimate() {
      return hasUpdated;
    }
  
    public Pose2d getEstimatedVisionBotPose(){
      return estimatedVisionBotPose;
    }
  
    public VisionConfig.CAMERA_TYPE getCameraType(){
      return cameraType;
    }

    public String getCameraName(){
      return photonCamera.getName();
    }
  
    public boolean isMultiTagEstimate(){
      return isMultiTag;
    }
  
    public void setDistanceToBestTarget(double bestDistance){
      distanceToBestTarget = bestDistance;
    }
  
    public double getDistanceToBestTarget() {
      return distanceToBestTarget;
    }
}
  