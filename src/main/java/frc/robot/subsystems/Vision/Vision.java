package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.VisionConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightTarget_Fiducial;

import java.text.DecimalFormat;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private Pose2d botPose;
  private Pose2d estimatePose;
  private double limeLatency;
  private boolean apriltagLimelightConnected = false;
  private boolean NNLimelightConnected = false;


  private double photonTimestamp;
  private PhotonCamera photonCam_1;
  private boolean photon1HasTargets;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonEstimator_1;

  private PhotonCamera photonCam_2;
  private boolean photon2HasTargets;
  private PhotonPoseEstimator photonEstimator_2;

  private PhotonCamera photonCam_3;
  private boolean photon3HasTargets;
  private PhotonPoseEstimator photonEstimator_3;

  private int primaryAprilTagID;
  private int primaryCameraNum;
  private double tempCutOff;
  private double ambiguityForCam;
  private double ambiguityForCam1;
  private double ambiguityForCam2;
  private double ambiguityForCam3;
  
  private double distToTagRelativeToRobot;
  private double distToTagRelativeToCam;

  // For Note detection in the future
  private double detectHorizontalOffset = 0;
  private double detectVerticalOffset = 0;

  private boolean detectTarget = false;
  private LimelightHelpers.LimelightResults jsonResults, detectJsonResults;
  private Pose2d targetRobotRelativePose;
  private Pose2d noteFieldRelativePose;
  private Pose2d noteRobotRelativePose;
  private ShuffleboardTab tab = Shuffleboard.getTab("Vision Data");
  private GenericEntry visionAmbiguity = tab.add("Vision Ambiguity", VisionConfig.POSE_AMBIGUITY_CUTOFF).getEntry();

  //Pose Filtering
  private Pose2d filteredVisionPose;
  private LinearFilter xFilter; 
  private LinearFilter yFilter; 
  private LinearFilter thetaFilter; 
  private boolean isVisionEstimatePoseChanged;

  // testing
  private final DecimalFormat df = new DecimalFormat();

  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) instance = new Vision();
    return instance;
  }

  // TODO - see if adding setCameraPose_RobotSpace() is needed from LimelightHelpers
  private Vision() {
    distToTagRelativeToCam = -1;
    distToTagRelativeToRobot = -1;
    ambiguityForCam = -1;
    ambiguityForCam1 = -1;
    ambiguityForCam2 = -1;
    ambiguityForCam3 = -1;
    tempCutOff = 0.1;
    primaryAprilTagID = -1;
    primaryCameraNum = -1;
    setName("Vision");
    botPose = new Pose2d();
    estimatePose = new Pose2d();
    noteFieldRelativePose = new Pose2d();
    noteRobotRelativePose = new Pose2d();
    noteRobotRelativePose = new Pose2d();
    targetRobotRelativePose = new Pose2d();
    photonTimestamp = 0.0;
    limeLatency = 0.0;

    // Creates Filters for x, y and Theta
    xFilter = LinearFilter.movingAverage(VisionConfig.MOVING_AVG_TAPS);
    yFilter = LinearFilter.movingAverage(VisionConfig.MOVING_AVG_TAPS);
    thetaFilter = LinearFilter.movingAverage(VisionConfig.MOVING_AVG_TAPS);
    isVisionEstimatePoseChanged = false;

    // Changes vision mode between limelight and photonvision for easy switching
    if (VisionConfig.IS_LIMELIGHT_APRILTAG_MODE) {
      // configure both limelights
      LimelightHelpers.setLEDMode_ForceOn(VisionConfig.POSE_LIMELIGHT);
      setLimelightPipeline(VisionConfig.POSE_LIMELIGHT, VisionConfig.APRILTAG_PIPELINE);
      LimelightHelpers.setCameraPose_RobotSpace(
          VisionConfig.POSE_LIMELIGHT,
          VisionConfig.POSE_LIME_X,
          VisionConfig.POSE_LIME_Y,
          VisionConfig.POSE_LIME_Z,
          VisionConfig.POSE_LIME_ROLL,
          VisionConfig.POSE_LIME_PITCH,
          VisionConfig.POSE_LIME_YAW);
    }

    if (VisionConfig.IS_NEURAL_NET_LIMELIGHT) {
        LimelightHelpers.setLEDMode_ForceOff(VisionConfig.NN_LIMELIGHT);
        setLimelightPipeline(VisionConfig.NN_LIMELIGHT, VisionConfig.NOTE_DETECTOR_PIPELINE);
      }

    //Code to make the first photon vision camera object
    if (VisionConfig.IS_PHOTON_VISION_ENABLED) { // Configure photonvision camera
      photonCam_1 = new PhotonCamera(VisionConfig.POSE_PHOTON_1);
      //photonCam_2 = new PhotonCamera(VisionConfig.POSE_PHOTON_2);
      photon1HasTargets = false;

      try {
        aprilTagFieldLayout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        System.out.println("Field layout not found");
      }
      photonEstimator_1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConfig.PHOTON_1_ROBOT_TO_CAM);
      photonEstimator_1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    //Code to make the second photon vision camera object if it is enabled
    if (VisionConfig.IS_PHOTON_VISION_ENABLED && VisionConfig.IS_PHOTON_TWO_ENABLED) {
      photonCam_2 = new PhotonCamera(VisionConfig.POSE_PHOTON_2);
      photon2HasTargets = false;
      photonEstimator_2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConfig.PHOTON_2_ROBOT_TO_CAM);
      photonEstimator_2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    //Code to make the second photon vision camera object if it is enabled
    if (VisionConfig.IS_PHOTON_VISION_ENABLED && VisionConfig.IS_PHOTON_THREE_ENABLED) {
      photonCam_3 = new PhotonCamera(VisionConfig.POSE_PHOTON_3);
      photon3HasTargets = false;
      photonEstimator_3 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConfig.PHOTON_3_ROBOT_TO_CAM);
      photonEstimator_3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    

    if (VisionConfig.DRIVER_CAMERA_ACTIVE){
      tab.addCamera("Driver Camera", "Drive cam", VisionConfig.DRIVER_CAM_STREAM);
    }

    // printing purposes
    df.setMaximumFractionDigits(2);

    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Subsystem");
    visionTab.add(this);
  }

  @Override
  public void periodic() {
    // 8.308467, 1.442593 and 1.451102
    // botPose
    PhotonTrackedTarget target = photonCam_3.getLatestResult().getBestTarget();
    if(target != null){
    distToTagRelativeToRobot = getTargetDistanceFromRobot(target.getBestCameraToTarget());
    distToTagRelativeToCam = getTargetDistanceFromCamera(target.getBestCameraToTarget(), 3);}
    tempCutOff = visionAmbiguity.getDouble(0.0);

    /*Ensures empty json not fed to pipeline*/
    apriltagLimelightConnected =
        !NetworkTableInstance.getDefault()
            .getTable(VisionConfig.POSE_LIMELIGHT)
            .getEntry("json")
            .getString("")
            .equals("");

    NNLimelightConnected =
        !NetworkTableInstance.getDefault()
            .getTable(VisionConfig.NN_LIMELIGHT)
            .getEntry("json")
            .getString("")
            .equals("");
      
    if (VisionConfig.IS_LIMELIGHT_APRILTAG_MODE && apriltagLimelightConnected) {
      jsonResults = LimelightHelpers.getLatestResults(VisionConfig.POSE_LIMELIGHT);

      estimatePose = LimelightHelpers.getBotPose2d_wpiBlue(VisionConfig.POSE_LIMELIGHT);

      if (visionAccurate(estimatePose)) {
        // Blue alliance means origin is bottom right of the field 
        limeLatency =
            LimelightHelpers.getLatency_Pipeline(VisionConfig.POSE_LIMELIGHT)
                + LimelightHelpers.getLatency_Capture(VisionConfig.POSE_LIMELIGHT);
        botPose = estimatePose;
        isVisionEstimatePoseChanged = true;
      }
    }

    Pose2d latestVisionPose = getCameraPrioritizedPose();
    if(latestVisionPose != null){
      botPose = latestVisionPose;
    }

    // Filtering x, y and theta
    if(isVisionEstimatePoseChanged){
      filteredVisionPose = new Pose2d(xFilter.calculate(botPose.getX()), 
                                      yFilter.calculate(botPose.getY()), 
                                      Rotation2d.fromDegrees(thetaFilter.calculate(botPose.getRotation().getDegrees())));
    }

    //Does math to see where the note is
    if (VisionConfig.IS_NEURAL_NET_LIMELIGHT && NNLimelightConnected) {
      detectTarget = LimelightHelpers.getTV(VisionConfig.NN_LIMELIGHT);
      detectJsonResults = LimelightHelpers.getLatestResults(VisionConfig.NN_LIMELIGHT);
      //var rrResults = detectJsonResults.targetingResults.targets_Retro[0];

      if (detectTarget) {
        detectHorizontalOffset = -LimelightHelpers.getTX(VisionConfig.NN_LIMELIGHT); //HAD TO NEGATIVE TO MAKE CCW POSITIVE
        detectVerticalOffset = LimelightHelpers.getTY(VisionConfig.NN_LIMELIGHT);
        double targetDist = targetDistanceMetersCamera(VisionConfig.NN_LIME_Z, VisionConfig.NN_LIME_PITCH, 0, detectVerticalOffset);
        //Note: limelight is already CCW positive, so tx does not have to be * -1
        Translation2d camToTargTrans = estimateCameraToTargetTranslation(targetDist, detectHorizontalOffset);

        //Code for robot relative note tracking
        Transform2d robotToNoteTransform = VisionConfig.NN_ROBOT_TO_LIME_2D.plus(new Transform2d(camToTargTrans, Rotation2d.fromDegrees(0.0)));
        Rotation2d targetAngleRobotRelative = robotToNoteTransform.getTranslation().getAngle().plus(new Rotation2d(Math.PI));
        noteRobotRelativePose = new Pose2d(robotToNoteTransform.getTranslation(), targetAngleRobotRelative);

        //Code for field relative note tracking
        Pose2d currentBotPoseFieldRelative = PoseEstimator.getInstance().getPosition();

        Pose2d camPoseFieldRelative = currentBotPoseFieldRelative.plus(VisionConfig.NN_ROBOT_TO_LIME_2D);
        noteFieldRelativePose = camPoseFieldRelative.plus(new Transform2d(camToTargTrans, Rotation2d.fromDegrees(0.0)));
        Translation2d currentBotTranslation = currentBotPoseFieldRelative.getTranslation();
        Translation2d targetVector = currentBotTranslation.minus(noteFieldRelativePose.getTranslation());
        Rotation2d targetAngle = targetVector.getAngle();
        
        noteFieldRelativePose = new Pose2d(noteFieldRelativePose.getTranslation(), targetAngle);
      }
    }
    photon1HasTargets = photonCam_1.getLatestResult().hasTargets();
    photon2HasTargets = photonCam_2.getLatestResult().hasTargets();
    photon3HasTargets = photonCam_3.getLatestResult().hasTargets();

  }

  public Pose2d getCameraPrioritizedPose(){
    Pose2d newBotPose = null;

    PhotonPipelineResult[] cameras = {photonCam_3.getLatestResult(), 
                                    photonCam_1.getLatestResult(), 
                                    photonCam_2.getLatestResult()
                                  };
    PhotonTrackedTarget a1 = photonCam_1.getLatestResult().getBestTarget();
    PhotonTrackedTarget a2 = photonCam_2.getLatestResult().getBestTarget();
    PhotonTrackedTarget a3 = photonCam_3.getLatestResult().getBestTarget();        
                        
    if(a1 != null) ambiguityForCam1 = a1.getPoseAmbiguity();
    if(a2 != null) ambiguityForCam2 = a2.getPoseAmbiguity();
    if(a3 != null) ambiguityForCam3 = a3.getPoseAmbiguity();
    isVisionEstimatePoseChanged = false; 

    Transform3d[] camToRobotArray = {VisionConfig.PHOTON_3_CAM_TO_ROBOT, VisionConfig.PHOTON_1_CAM_TO_ROBOT, VisionConfig.PHOTON_2_CAM_TO_ROBOT};
    PhotonPipelineResult selectedCameraResult = null;
    double lowestAmbiguity = 1;
    int chosenCameraNum = 0;
    boolean bestCaseIsMultiTag = false;

    // go through the cameras and select the one with multi target and lowest ambiguity
    for(int i = 0; i < cameras.length; i++){
      if (cameras[i] != null && cameras[i].hasTargets()) { // if the camera is enabled and has a target, consider it
        if (cameras[i].getMultiTagResult().estimatedPose.isPresent && ((cameras[i].getBestTarget().getPoseAmbiguity() >=0 && // if the camera has a multi tag pose
            cameras[i].getBestTarget().getPoseAmbiguity() < tempCutOff) || i==0)) { // and the ambiguity is low enough
            
              // Get the ambiguity value of the camera
              double ambiguity = cameras[i].getBestTarget().getPoseAmbiguity();
              // Check if the ambiguity value is lower than the current lowest ambiguity
              
              if (ambiguity < lowestAmbiguity) {
                // Update the selected camera and lowest ambiguity value
                selectedCameraResult = cameras[i];
                lowestAmbiguity = ambiguity;
                bestCaseIsMultiTag = true;
                chosenCameraNum = i;

              }
            }
          }
        }

      //if we didn't get a camera with a multi tag pose, we will use the camera with the lowest ambiguity
      if (selectedCameraResult == null){
        for (int i = 0; i < cameras.length; i++) {
          if (cameras[i] != null && cameras[i].hasTargets()) { // if the camera is enabled and has a target, consider it
            if ((cameras[i].getBestTarget().getPoseAmbiguity() < tempCutOff && cameras[i].getBestTarget().getPoseAmbiguity() >= 0)||i==0) { // the ambiguity is low enough
                
                  // Get the ambiguity value of the camera
                  double ambiguity = cameras[i].getBestTarget().getPoseAmbiguity();
                  // Check if the ambiguity value is lower than the current lowest ambiguity
                  
                  if (ambiguity < lowestAmbiguity) {
                    // Update the selected camera and lowest ambiguity value
                    //if(cameras[0].getBestTarget().getBestCameraToTarget().getX() )
                    selectedCameraResult = cameras[i];
                    lowestAmbiguity = ambiguity;
                    chosenCameraNum = i;
                  }
                }
              }
            }
      }

      //Area cutoff

    // if a camera has been selected, we will use it to get the pose
    if (selectedCameraResult != null && lowestAmbiguity < tempCutOff){
      photonTimestamp = selectedCameraResult.getTimestampSeconds();
      PhotonTrackedTarget target = selectedCameraResult.getBestTarget();

      if (bestCaseIsMultiTag) {
        Transform3d fieldToCamera = selectedCameraResult.getMultiTagResult().estimatedPose.best;
        Transform3d fieldCamToRobot = fieldToCamera.plus(camToRobotArray[chosenCameraNum]);
        Pose3d botPose3d = new Pose3d(fieldCamToRobot.getX(), fieldCamToRobot.getY(), fieldCamToRobot.getZ(), fieldCamToRobot.getRotation());
        newBotPose = botPose3d.toPose2d();
        ambiguityForCam = target.getPoseAmbiguity(); 
      } 
      else {
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
        Pose3d currentPose3d = PhotonUtils.estimateFieldToRobotAprilTag(bestCameraToTarget, tagPose, 
                            camToRobotArray[chosenCameraNum]); // this needs to be universalized, right now is fixed to photon 3
        newBotPose = currentPose3d.toPose2d();
        ambiguityForCam = target.getPoseAmbiguity(); 
      }

      isVisionEstimatePoseChanged = true;
      primaryAprilTagID = target.getFiducialId();
      if(chosenCameraNum == 0) primaryCameraNum = 3; else if(chosenCameraNum == 1) primaryCameraNum = 1; else primaryCameraNum = 2;
      
    }
    return newBotPose;
  }

  /**
   * Update all PhotonPoseEstimators
   */
  public void updateAllPhotonPoseEstimators(){
    PhotonPoseEstimator[] photonPoseEstimators = {photonEstimator_1, photonEstimator_2, photonEstimator_3};
    

  }
  

  /**
   * 
   * @return whether the limelight currently sees a game piece
   */
  public boolean gamePieceDetected(){
    return detectTarget;
  }

  /**
   * @return Pose2d location of note Field Relative
   */
  public Pose2d getNotePose2d(){
    return noteFieldRelativePose;
  }

  /**
   * @return Pose2d location of note Field Relative
   */
  public Pose2d getRobotRelativeNotePose2d(){
    return noteRobotRelativePose;
  }


  /**
   * @return Timestamp of photonvision's latest reading
   */
  public double getPhotonTimestamp() {
    return photonTimestamp;
  }

  /**
   * @return boolean if photonvision has targets
   */
  public boolean photonHasTargets() {
    return photon1HasTargets||photon2HasTargets||photon3HasTargets;
  }

  /**
   * @return boolean if bot pose has changed due to vision in the last refresh of periodic
   */
  public boolean isVisionEstimatePoseChanged(){
    return isVisionEstimatePoseChanged;
  }
  
  /**
   * @return RobotPose2d with the apriltag as the origin (for chase apriltag command)
   */
  public Pose2d getRobotPose2d_TargetSpace() {
    return LimelightHelpers.getBotPose2d_TargetSpace(VisionConfig.POSE_LIMELIGHT);
  }

  /**
   * @return Pose2d of the apriltag with the robot as the origin
   */
  public Pose2d getTargetRobotPose_RobotSpace() {
    return LimelightHelpers.getTargetPose2d_RobotSpace(VisionConfig.POSE_LIMELIGHT);
  }

  /**
   * @return 3D distance to tag
   */
  public double getTargetDistanceFromRobot(Transform3d targetPose) {
    return Math.sqrt(Math.pow(targetPose.getX() + botPose.getX(),2) + Math.pow(targetPose.getY() + botPose.getY(),2) + Math.pow(targetPose.getZ(),2));
  }

 public double getTargetDistanceFromCamera(Transform3d targetPose, int camNum) {
  Transform3d camPose;
  if(camNum == 1){camPose = new Transform3d(VisionConfig.CAM_1_X,VisionConfig.CAM_1_Y ,VisionConfig.CAM_1_Z, new Rotation3d()); }
  if(camNum == 2){camPose = new Transform3d(VisionConfig.CAM_2_X,VisionConfig.CAM_2_Y ,VisionConfig.CAM_2_Z, new Rotation3d()); }
  if(camNum == 3){camPose = new Transform3d(VisionConfig.CAM_3_X,VisionConfig.CAM_3_Y ,VisionConfig.CAM_3_Z, new Rotation3d()); }
  else camPose = new Transform3d(0.0,0.0,0.0,new Rotation3d());
    return Math.sqrt(Math.pow(targetPose.getX() + (camPose.getX()+botPose.getX()),2) + Math.pow(targetPose.getY() + (camPose.getY()+botPose.getY()),2) + Math.pow(targetPose.getZ() + camPose.getZ(),2));
  }

  // APRILTAG HELPER METHODS

  /**
   * @return if vision should be trusted more than estimated pose
   */
  public boolean visionAccurate(Pose2d currentPose) {
    return isVisionEstimatePoseChanged && isInMap(currentPose);
  }

  /**
   * @return whether or not vision sees a tag
   */
  public boolean isValidPose() {
    /* Disregard Vision if there are no targets in view */
    if (VisionConfig.IS_LIMELIGHT_APRILTAG_MODE) {
      return LimelightHelpers.getTV(VisionConfig.POSE_LIMELIGHT);
    }
    if (VisionConfig.IS_PHOTON_VISION_ENABLED) {
      return photonHasTargets();
    }
    return false;
  }

  // This is a suss function - need to test it
  public boolean isInMap(Pose2d currentPose) {
    return ((currentPose.getX() >= 0.0 && currentPose.getX() <= VisionConfig.FIELD_LENGTH_METERS)
        && (currentPose.getY() >= 0.0 && currentPose.getY() <= VisionConfig.FIELD_WIDTH_METERS));
  }

  /**
   * @return whether the camera sees multiple tags or not
   */
  public boolean multipleTargetsInView() {
    if (jsonResults == null) {
      return false;
    }
    LimelightTarget_Fiducial[] tags = jsonResults.targetingResults.targets_Fiducials;
    if (tags.length > 1) {
      return true;
    }
    return false;
  }

  // Getter for visionBotPose - NEED TO DO TESTING TO MAKE SURE NO NULL ERRORS

  public Pose2d visionBotPose() {
    return botPose;
  }

  public Pose2d getFilterVisionPose(){
    return filteredVisionPose;

  }

  /**
   * @return the total latency of the limelight camera
   */
  public double getTotalLatency() {
    return limeLatency;
  }

  /**
   * Gets the camera capture time in seconds. Only used for limelight
   *
   * @param latencyMillis the latency of the camera in milliseconds
   * @return the camera capture time in seconds
   */
  public double getTimestampSeconds(double latencyMillis) {
    return Timer.getFPGATimestamp() - (latencyMillis / 1000d);
  }

  /**
   * @param limelight name of limelight to control in {@link VisionConfig}
   * @param pipelineIndex use pipeline indexes in {@link VisionConfig}
   */
  public void setLimelightPipeline(String limelight, int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(limelight, pipelineIndex);
  }

  /**
   * Gets target distance from the camera
   * @param cameraHeight distance from lens to floor of camera in meters
   * @param cameraAngle pitch of camera in radians
   * @param targetHeight distance from floor to center of target in meters
   * @param targetOffsetAngle_Vertical ty entry from limelight of target crosshair (in degrees)
   * @return the distance to the target in meters
   */
  public double targetDistanceMetersCamera(
      double cameraHeight,
      double cameraAngle,
      double targetHeight,
      double targetOffsetAngle_Vertical) {
    double angleToGoalRadians = cameraAngle + targetOffsetAngle_Vertical * (3.14159 / 180.0);
    return (targetHeight - cameraHeight) / Math.tan(angleToGoalRadians);
  }

   /**
   * @param targetDistanceMeters component of distance from camera to target
   * @param targetOffsetAngle_Horizontal tx entry from limelight of target crosshair (in degrees)
   * @return the translation to the target in meters
   */
  public Translation2d estimateCameraToTargetTranslation(double targetDistanceMeters, double targetOffsetAngle_Horizontal){
    Rotation2d yaw = Rotation2d.fromDegrees(targetOffsetAngle_Horizontal);
    return new Translation2d(
      yaw.getCos() * (targetDistanceMeters), yaw.getSin() * targetDistanceMeters);
  }
/**
   * @param cameraToTargetTranslation2d the translation from estimate camera to target
   * @param targetOffsetAngle_Horizontal tx entry from limelight of target crosshair (in degrees)
   * @return the position of the target in terms of the camera
   */
  public Pose2d estimateCameraToTargetPose2d(Translation2d cameraToTargetTranslation2d, double targetOffsetAngle_Horizontal){
    return new Pose2d(cameraToTargetTranslation2d, Rotation2d.fromDegrees(targetOffsetAngle_Horizontal));
  }


/**
   * @param camToTargetPose the camera to target pose 2d
   * @param camToRobot the transform from the x and y of the camera to the center of the robot
   * @return the position of the target relative to the robot
   */
  public Pose2d camPoseToRobotRelativeTargetPose2d(Pose2d camToTargetPose, Transform2d camToRobot){
    return camToTargetPose.transformBy(camToRobot);
    
  }

  /**
   * RobotRelativePose of the current target
   * @return the position of the target relative to the robot
   */
  public Pose2d targetPoseRobotSpace(){
    return targetRobotRelativePose;
  }

  /**
   * @param notePoseRobotRelative the RobotRelative Pose2d of the note
   * @param botPoseFieldRelative The FieldRelative Pose2d of the robot
   * @return the FieldRelative Pose2d of the note
   */
  public Pose2d notePoseFieldSpace(Pose2d notePoseRobotRelative, Pose2d botPoseFieldRelative){
    Transform2d noteTransform = new Transform2d(notePoseRobotRelative.getTranslation(), notePoseRobotRelative.getRotation());
    Pose2d notePose = botPoseFieldRelative.transformBy(noteTransform);
    return notePose; 
  }

  /**
   * Commnad to go to the note
   * @return a follow path command to drive to the note
   */
  public Command onTheFlyToNoteCommand(){
    return Drivetrain.getInstance().onTheFlyPathCommand(this::getNotePose2d); //doing this::getNotePose2d converts to a supplier
  }

  /**
   * Commnad to go to in front of note using PID
   * @return a PID command to drive in front of a note
   */
  public Command PID_thenOnTheFlyToNoteCommand(){
    return Drivetrain.getInstance().chaseThenOnTheFlyCommand(this::getNotePose2d);
  }

  /**
   * Commnad to go to in front of note using PID and then drive to the note
   * @return a PID and then on-the-fly command to drive onto a note
   */
  public Command PIDtoNoteCommand(){
    return Drivetrain.getInstance().chasePoseCommand(this::getNotePose2d);
  }

  /**
   * Command that uses PID to drive to the note robot relative
   * @return a PID command to drive onto a note robot relative
   */
  public Command PIDtoNoteRobotRelativeCommand(){
    return Drivetrain.getInstance().chasePoseRobotRelativeCommand(this::getRobotRelativeNotePose2d);
  }

  public int isTarget1(){
    if (photon1HasTargets){
      return 1;
    }
    else{
      return 0;
    }
  }
  public int isTarget2(){
    if (photon2HasTargets){
      return 1;
    }
    else{
      return 0;
    }
  }
  public int isTarget3(){
    if (photon3HasTargets){
      return 1;
    }
    else{
      return 0;
    }
  }
  public int isEstimate(){
    if (isVisionEstimatePoseChanged){
      return 1;
    }
    else{
      return 0;
    }
  }
  public int hasValidTargetsInt(){
    if (photonHasTargets()){
      return 1;
    }
    else{
      return 0;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addBooleanProperty("Has valid AprilTag targets", () -> photonHasTargets(), null);
    builder.addBooleanProperty("Has Vision Pose changed", () -> isVisionEstimatePoseChanged(), null);

    builder.addBooleanProperty("cam 1 front has targets", () -> photon1HasTargets, null);
    builder.addBooleanProperty("cam 2 back has targets", () -> photon2HasTargets, null);
    builder.addBooleanProperty("cam 3 front laser has targets", () -> photon3HasTargets, null);
    // integer for Graphing
    builder.addIntegerProperty("Has valid AprilTag targets - int", () -> hasValidTargetsInt(), null);
    builder.addIntegerProperty("Has Vision Pose changed - int", () -> isEstimate(), null);

    builder.addIntegerProperty("cam 1 front has targets - int", () -> isTarget1(), null);
    builder.addIntegerProperty("cam 2 back has targets - int", () -> isTarget2(), null);
    builder.addIntegerProperty("cam 3 front laser has targets - int", () -> isTarget3(), null);
    
    builder.addIntegerProperty("Primary Tag Used For Odometry", ()->primaryAprilTagID, null);
    builder.addIntegerProperty("Primary Camera Used For Odometry", ()->primaryCameraNum, null);
    builder.addDoubleProperty("Ambiguity For Cam In Use", ()-> ambiguityForCam, null);
    
    builder.addDoubleProperty("cam 1 front ambiguity", ()->ambiguityForCam1, null);
    builder.addDoubleProperty("cam 2 back ambiguity", ()->ambiguityForCam2, null);
    builder.addDoubleProperty("cam 3 laser ambiguity", ()-> ambiguityForCam3, null);
    builder.addDoubleProperty("Distance to Tag - Robot", ()-> distToTagRelativeToRobot, null);
    builder.addDoubleProperty("Distance to Tag - Cam", ()-> distToTagRelativeToCam, null);

    

  }
}