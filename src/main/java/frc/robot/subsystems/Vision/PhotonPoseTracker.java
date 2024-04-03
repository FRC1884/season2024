package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotMap.VisionConfig;

public class PhotonPoseTracker {

    public PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera photonCamera;
    private boolean hasUpdatedPoseEstimate;
    private PhotonPipelineResult photonPipelineResult;
    private Pose2d estimatedVisionBotPose;
    private double visionEstimateTimestamp;
    private double distanceToBestTarget;
  
    private VisionConfig.CAMERA_TYPE cameraType;
    private boolean isMultiTag;
  
    public PhotonPoseTracker(PhotonPoseEstimator photonPoseEstimator, PhotonCamera photonCamera, VisionConfig.CAMERA_TYPE cameraType) {
      this.photonPoseEstimator = photonPoseEstimator;
      this.hasUpdatedPoseEstimate = false;
      this.photonCamera = photonCamera;
      estimatedVisionBotPose = new Pose2d();
      this.cameraType = cameraType;
      isMultiTag = false;
    }
  
    public void updateCameraPipelineResult(){
      photonPipelineResult = photonCamera.getLatestResult();
      visionEstimateTimestamp = photonPipelineResult.getTimestampSeconds();
      hasUpdatedPoseEstimate = false;
    }
  
    public PhotonPipelineResult getPhotonPipelineResult(){
      return photonPipelineResult;
    }
  
    public void setUpdatedStatus(boolean updateStatus){
      hasUpdatedPoseEstimate = updateStatus; 
    }
  
    public double getCurrentTimestamp(){
      return visionEstimateTimestamp;
    }
  
    public void updateEstimatedBotPose(){
      photonPoseEstimator.update(photonPipelineResult).ifPresent(estimatedRobotPose -> {
          Pose3d currentEstimatedPose = estimatedRobotPose.estimatedPose;
          estimatedVisionBotPose = currentEstimatedPose.toPose2d();
          isMultiTag = photonPipelineResult.getMultiTagResult().estimatedPose.isPresent;
          setDistanceToBestTarget(get3dDistance(photonPipelineResult.getBestTarget().getBestCameraToTarget()));
          hasUpdatedPoseEstimate = true;
        });
    }
  
    public boolean hasUpdatedVisionEstimate() {
      return hasUpdatedPoseEstimate;
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

    /**
   * @return 3D distance to tag
   */
  public double get3dDistance(Transform3d targetPose) {
    return Math.sqrt(Math.pow(targetPose.getX(),2) + Math.pow(targetPose.getY(),2) + Math.pow(targetPose.getZ(),2));
  }
}
  