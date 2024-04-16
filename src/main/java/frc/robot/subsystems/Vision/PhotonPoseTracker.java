package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotMap.VisionConfig;

/** Helper class that simplifies the process of updating and storing poses from PhotonVision*/
public class PhotonPoseTracker {

    //Photonvision variables
    public PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera photonCamera;
    private PhotonPipelineResult photonPipelineResult;

    //local variables
    private boolean hasUpdatedPoseEstimate;
    private Pose2d estimatedVisionBotPose;
    private double visionEstimateTimestamp;
    private double distanceToBestTarget;
    private double noisyDistanceMeters;
    private Pose3d visionPose3d;
  
    private VisionConfig.CAMERA_TYPE cameraType;
    private boolean isMultiTag;
    private boolean hasValidPose;
  
    public PhotonPoseTracker(PhotonPoseEstimator photonPoseEstimator, PhotonCamera photonCamera, VisionConfig.CAMERA_TYPE cameraType) {
      this.photonPoseEstimator = photonPoseEstimator;
      this.hasUpdatedPoseEstimate = false;
      this.photonCamera = photonCamera;
      estimatedVisionBotPose = new Pose2d();
      visionPose3d = new Pose3d();
      this.cameraType = cameraType;
      isMultiTag = false;
      distanceToBestTarget = 0;
      noisyDistanceMeters = 0;
      hasValidPose = false;
      switch (cameraType) { 
        case OV2311:
            noisyDistanceMeters = VisionConfig.OV2311_NOISY_DISTANCE_METERS;
            break;
        case OV9281:
            noisyDistanceMeters = VisionConfig.OV9281_NOISY_DISTANCE_METERS;
            break;
        case TELEPHOTO_OV9281:
            noisyDistanceMeters = VisionConfig.TELEPHOTO_NOISY_DISTANCE_METERS;
            break;
        default:
            break; 
          }
    }
  
    public void updateCameraPipelineResult(){
      photonPipelineResult = photonCamera.getLatestResult();
      visionEstimateTimestamp = photonPipelineResult.getTimestampSeconds();
      hasUpdatedPoseEstimate = false;
      hasValidPose = false;
    }

    public void updateEstimatedBotPose(){
      photonPoseEstimator.update(photonPipelineResult).ifPresent(estimatedRobotPose -> {
        visionPose3d = estimatedRobotPose.estimatedPose;
        estimatedVisionBotPose = visionPose3d.toPose2d();
        isMultiTag = photonPipelineResult.getMultiTagResult().estimatedPose.isPresent;
        setDistanceToBestTarget(get3dDistance(photonPipelineResult.getBestTarget().getBestCameraToTarget()));
        hasValidPose = true;
        });
        if (distanceToBestTarget < noisyDistanceMeters){
          hasUpdatedPoseEstimate = true;
        }
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

    public boolean hasUpdatedVisionEstimate() {
      return hasUpdatedPoseEstimate;
    }

    public boolean hasValidPose(){
      return hasValidPose;
    }
  
    public Pose2d getEstimatedVisionBotPose(){
      return estimatedVisionBotPose;
    }

    public Pose3d get3dEstimatedVisionPose(){
      return visionPose3d;
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
  