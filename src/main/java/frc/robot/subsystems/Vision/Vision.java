package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.VisionConfig;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightTarget_Fiducial;
import java.text.DecimalFormat;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/*
 * This class requires MAJOR CLEANUP. There needs to be a proper pyramid of hierarchy. Vision should NOT be able to control anything related to pose. It should only
 * broadcast its current pose, if it has one, for use by the Pose class. Vision --> Pose. Vision should NEVER be able to control robot odometry.
 *
 */

public class Vision extends SubsystemBase {
  private Pose2d botPose;
  private double limeLatency;
  private boolean apriltagLimelightConnected = false;
  private boolean NNLimelightConnected = false;

  private double photonTimestamp;
  private PhotonCamera photonCam_1;
  private boolean photon1HasTargets;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;
  private Transform3d robotToCam;

  // For Note detection in the future
  private double detectHorizontalOffset = 0;
  private double detectVerticalOffset = 0;

  private int targetSeenCount = 0;

  private boolean aimTarget = false;
  private boolean detectTarget = false;
  private LimelightHelpers.LimelightResults jsonResults, detectJsonResults;

  // testing
  private final DecimalFormat df = new DecimalFormat();

  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) instance = new Vision();
    return instance;
  }

  // TODO - see if adding setCameraPose_RobotSpace() is needed from LimelightHelpers
  private Vision() {
    setName("Vision");
    botPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));
    photonTimestamp = 0.0;
    limeLatency = 0.0;
    // botPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // targetSeenCount = 0;
    // aimHorizontalOffset = 0;
    // aimVerticalOffset = 0;

    // Changes vision mode between limelight and photonvision for easy switching
    if (VisionConfig.isLimelightMode) {
      // configure both limelights
      LimelightHelpers.setLEDMode_ForceOff(VisionConfig.POSE_LIMELIGHT);
      setLimelightPipeline(VisionConfig.POSE_LIMELIGHT, VisionConfig.aprilTagPipeline);
      LimelightHelpers.setCameraPose_RobotSpace(
          VisionConfig.POSE_LIMELIGHT,
          VisionConfig.POSE_LIME_X,
          VisionConfig.POSE_LIME_Y,
          VisionConfig.POSE_LIME_Z,
          VisionConfig.POSE_LIME_ROLL,
          VisionConfig.POSE_LIME_PITCH,
          VisionConfig.POSE_LIME_YAW);

      if (VisionConfig.isNeuralNet) {
        LimelightHelpers.setLEDMode_ForceOff(VisionConfig.NN_LIMELIGHT);
        setLimelightPipeline(VisionConfig.NN_LIMELIGHT, VisionConfig.noteDetectorPipeline);
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConfig.NN_LIMELIGHT,
            VisionConfig.NN_LIME_X,
            VisionConfig.NN_LIME_Y,
            VisionConfig.NN_LIME_Z,
            VisionConfig.NN_LIME_ROLL,
            VisionConfig.NN_LIME_PITCH,
            VisionConfig.NN_LIME_YAW);
      }
    }
    if (VisionConfig.isPhotonVisionMode) { // Configure photonvision camera
      photonCam_1 = new PhotonCamera(VisionConfig.POSE_PHOTON_1);
      photon1HasTargets = false;
      try {
        aprilTagFieldLayout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        System.out.println("Field layout not found");
      }
      // Mounting information of photoncamera for making PhotonPoseEstimator object
      robotToCam =
          new Transform3d(
              new Translation3d(VisionConfig.CAM_1_X, VisionConfig.CAM_1_Y, VisionConfig.CAM_1_Z),
              new Rotation3d(
                  VisionConfig.CAM_1_ROLL_RADIANS,
                  VisionConfig.CAM_1_PITCH_RADIANS,
                  VisionConfig.CAM_1_YAW_RADIANS));
      // TODO for 9th graders - create PhotonPoseEstimator object
      photonPoseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              photonCam_1,
              robotToCam);
    }

    // printing purposes
    df.setMaximumFractionDigits(2);
  }

  @Override
  public void periodic() {
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

    if (VisionConfig.isLimelightMode && apriltagLimelightConnected) {
      if (visionAccurate()) {
        jsonResults = LimelightHelpers.getLatestResults(VisionConfig.POSE_LIMELIGHT);
        // json dump more accurate?
        // Update Vision robotpose - need to read more about coordinate systems centered
        // Blue alliance means origin is bottom right of the field
        botPose = LimelightHelpers.getBotPose2d_wpiBlue(VisionConfig.POSE_LIMELIGHT);
        limeLatency =
            LimelightHelpers.getLatency_Pipeline(VisionConfig.POSE_LIMELIGHT)
                + LimelightHelpers.getLatency_Capture(VisionConfig.POSE_LIMELIGHT);
      }

      // aimHorizontalOffset = jsonResults.results.getTX();
      // aimVerticalOffset = jsonResults.getTY();
      // aimTarget = LimelightHelpers.getTV(VisionConfig.POSE_LIMELIGHT);

      // Robot.log.logger.recordOutput("aimLL-VertOffset", aimVerticalOffset);
      // RobotTelemetry.print("aimLL-VertOffset: " + aimVerticalOffset);
    }

    if (NNLimelightConnected) {
      detectTarget = LimelightHelpers.getTV(VisionConfig.NN_LIMELIGHT);
      detectJsonResults = LimelightHelpers.getLatestResults(VisionConfig.NN_LIMELIGHT);
      if (detectTarget) {
        detectHorizontalOffset = LimelightHelpers.getTX(VisionConfig.NN_LIMELIGHT);
        detectVerticalOffset = LimelightHelpers.getTY(VisionConfig.NN_LIMELIGHT);
      }
    }
    // this method can call update() if vision pose estimation needs to be updated in
    // Vision.java

    // Photonvision Result
    // The documentation for this is here:
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // The example code was missing, and we came up with this:
    if (VisionConfig.isPhotonVisionMode) {
      var result = photonCam_1.getLatestResult();
      photon1HasTargets = result.hasTargets();
      if (photon1HasTargets) {
        var update = photonPoseEstimator.update();
        Pose3d currentPose3d = update.get().estimatedPose;
        botPose = currentPose3d.toPose2d();
        photonTimestamp = update.get().timestampSeconds;
      }
    }
  }

  public double getPhotonTimestamp() {
    return photonTimestamp;
  }

  public boolean photonHasTargets() {
    return photon1HasTargets;
  }

  public Pose2d getRobotPose2d_TargetSpace() {
    return LimelightHelpers.getBotPose2d_TargetSpace(VisionConfig.POSE_LIMELIGHT);
  }

  /**
   * @return Pose2d of the apriltag with the robot as the origin
   */
  public Pose2d getTargetRobotPose_RobotSpace() {
    return LimelightHelpers.getTargetPose2d_RobotSpace(VisionConfig.POSE_LIMELIGHT);
  }

  // APRILTAG HELPER METHODS

  /**
   * @return if vision should be trusted more than estimated pose
   */
  public boolean visionAccurate() {
    return isValidPose() && (isInMap() || multipleTargetsInView());
  }

  /**
   * @return whether or not vision sees a tag
   */
  public boolean isValidPose() {
    /* Disregard Vision if there are no targets in view */
    if (VisionConfig.isLimelightMode) {
      return LimelightHelpers.getTV(VisionConfig.POSE_LIMELIGHT);
    }
    if (VisionConfig.isPhotonVisionMode) {
      return photonHasTargets();
    }
    return false;
  }

  // This is a suss function - need to test it
  public boolean isInMap() {
    return ((botPose.getX() > 0.0 && botPose.getX() <= VisionConfig.FIELD_LENGTH_METERS)
        && (botPose.getY() > 0.0 && botPose.getY() <= VisionConfig.FIELD_WIDTH_METERS));
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

  public double getTotalLatency() {
    return limeLatency;
  }

  /**
   * Gets the camera capture time in seconds.
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
   * @param cameraHeight distance from lens to floor of camera in meters
   * @param cameraAngle pitch of camera in radians
   * @param targetHeight distance from floor to center of target in meters
   * @param targetOffsetAngle_Vertical ty entry from limelight of target crosshair (in degrees)
   * @return the distance to the target in meters
   */
  public double targetDistanceMeters(
      double cameraHeight,
      double cameraAngle,
      double targetHeight,
      double targetOffsetAngle_Vertical) {
    double angleToGoalRadians = cameraAngle + targetOffsetAngle_Vertical * (3.14159 / 180.0);
    return (targetHeight - cameraHeight) / Math.tan(angleToGoalRadians);
  }

  /**
   * Prints the vision, estimated, and odometry pose to SmartDashboard
   *
   * @param values the array of limelight raw values
   */
  /*
  public void printDebug(double[] poseArray) {
      if (poseArray.length > 0) {
          SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
          SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
          SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
          SmartDashboard.putString(
                  "LimelightRoll",
                  df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
          SmartDashboard.putString(
                  "LimelightPitch",
                  df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
          SmartDashboard.putString(
                  "LimelightYaw",
                  df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
      }
      SmartDashboard.putString("EstimatedPoseX", df.format(Robot.pose.getEstimatedPose().getX()));
      SmartDashboard.putString("EstimatedPoseY", df.format(Robot.pose.getEstimatedPose().getY()));
      SmartDashboard.putString(
              "EstimatedPoseTheta", df.format(Robot.pose.getHeading().getDegrees()));
  }
   */
}
