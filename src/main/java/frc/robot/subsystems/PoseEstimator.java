package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PoseConfig;
import frc.robot.RobotMap.VisionConfig;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.subsystems.Vision.Vision;

import java.util.function.Supplier;

/** Reports our expected, desired, and actual poses to dashboards */
public class PoseEstimator extends SubsystemBase {
  private static PoseEstimator instance;

  public static PoseEstimator getInstance() {
    if (instance == null) instance = new PoseEstimator();
    return instance;
  }

  // PoseConfig config;
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatePose = new Pose2d();

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Drivetrain drivetrain;
  //private final KalmanFilter<N1, N1, N1> visionKalmanFilter;

  private ShuffleboardTab tab = Shuffleboard.getTab("Odometry Data");
  private GenericEntry xPoseDiffEntry = tab.add("XOdom Diff", 0).getEntry();
  private GenericEntry yPoseDiffEntry = tab.add("YODom Diff", 0).getEntry();
  private GenericEntry totalDiffEntry = tab.add("totalDiff", 0).getEntry();
  private GenericEntry rToSpeaker = tab.add("Distance to Speaker", 0).getEntry();
  private GenericEntry aprilTagTelemEntry = tab.add("Has AprilTag Telemetry", false).getEntry();
  private GenericEntry enableVisionOverride = tab.add("Vision override enabled", false).getEntry();
  

  private PoseEstimator() {
    // config = new PoseConfig();
    drivetrain = Drivetrain.getInstance();

    // Maxswerve Version from MAXSwerve.java in core
    poseEstimator =
        new SwerveDrivePoseEstimator(
            MaxSwerveConstants.kDriveKinematics,
            drivetrain.getYawRot2d(),
            drivetrain.getModulePositions(),
            drivetrain.getPose(),
            createStateStdDevs(
                PoseConfig.kPositionStdDevX,
                PoseConfig.kPositionStdDevY,
                PoseConfig.kPositionStdDevTheta),
            createVisionMeasurementStdDevs(
                PoseConfig.kVisionStdDevX,
                PoseConfig.kVisionStdDevY,
                PoseConfig.kVisionStdDevTheta));
    
    //Kalman vision filter
    //visionKalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N2(), Nat.N3(), );
  }

  @Override
  public void periodic() {
    updateOdometryEstimate(); // Updates using wheel encoder data only
    // Updates using the vision estimate
    Pose2d tempEstimatePose = Vision.getInstance().visionBotPose();
    if (VisionConfig.IS_LIMELIGHT_APRILTAG_MODE && tempEstimatePose != null
        && (tempEstimatePose.getX() > VisionConfig.VISION_X_MAX_CUTOFF || tempEstimatePose.getX() < VisionConfig.VISION_X_MIN_CUTOFF)) { // Limelight mode
      if (isEstimateReady(tempEstimatePose)) { // Does making so many bot pose variables impact accuracy?
        double currentTimestamp = Vision.getInstance().getTimestampSeconds(Vision.getInstance().getTotalLatency());
        addVisionMeasurement(tempEstimatePose, currentTimestamp);
      }
    }
    // TODO Photonvision mode - Needs editing and filtering
    if (VisionConfig.IS_PHOTON_VISION_ENABLED && tempEstimatePose != null) { 
      double photonTimestamp = Vision.getInstance().getPhotonTimestamp();
      if (isEstimateReady(tempEstimatePose)) { // Does making so many bot pose variables impact accuracy?
        addVisionMeasurement(tempEstimatePose, photonTimestamp);
        aprilTagTelemEntry.setBoolean(true);
      }
      if (enableVisionOverride.getBoolean(false)){
        addVisionMeasurement(tempEstimatePose, photonTimestamp);
        aprilTagTelemEntry.setBoolean(true);
      }
      else{
        aprilTagTelemEntry.setBoolean(false);
      }
    }

    //UNTESTED - ALWAYS SETS DRIVETRAIN ODOMETRY TO THE POSE-ESTIMATOR ODOMETRY
    //NOT GREAT FOR ERROR CHECKING POSE ESTIMATOR! - SET TO FALSE
    if (VisionConfig.VISION_OVERRIDE_ENABLED) {
      drivetrain.resetOdometry(getPosition());
    }

    // Update for telemetry
    setEstimatedPose(getPosition());
    setOdometryPose(drivetrain.getPose());

    double xDiff = estimatePose.getX() - odometryPose.getX();
    double yDiff = estimatePose.getY() - odometryPose.getY();

    xPoseDiffEntry.setDouble(xDiff);
    yPoseDiffEntry.setDouble(yDiff);
    totalDiffEntry.setDouble(Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)));

    double xAvg = (estimatePose.getX() + odometryPose.getX()) / 2;
    double yAvg = (estimatePose.getY() + odometryPose.getY()) / 2;
    drivetrain.resetOdometry(new Pose2d(xAvg, yAvg, drivetrain.getYawRot2d()));

    Translation2d currentTranslation = getPosition().getTranslation();

    if (DriverStation.getAlliance().isEmpty()) {
      return;
    }

    Pose2d targetCoordinate = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;

    double targetVectorLength = currentTranslation.getDistance(targetCoordinate.getTranslation());
    rToSpeaker.setDouble(targetVectorLength);

  }
  
  public Double getDistanceToPose(Translation2d pose) {
        return getPosition().getTranslation().getDistance(pose);
  }

  public Double getDistanceToPose(Supplier<Translation2d> pose) {
        return getPosition().getTranslation().getDistance(pose.get());
  }

  /**
   * Helper method for comparing vision pose against odometry pose. Does not account for difference
   * in rotation. Will return false vision if it sees no targets or if the vision estimated pose is
   * too far from the odometry estimate
   *
   * @return whether or not pose should be added to estimate or not
   */
  public boolean isEstimateReady(Pose2d pose) {
    /* Disregard Vision if there are no targets in view */
    if (!Vision.getInstance().visionAccurate(pose)) { // visionAccurate method sees if Apriltags present in Vision.java
      return false;
    }
    return true;
  }

  /** Sets the Odometry Pose to the given pose */
  public void setOdometryPose(Pose2d pose) {
    odometryPose = pose;
  }

  /** Returns the Odometry Pose from drivetrain */
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Sets the estimated pose to the given pose */
  public void setEstimatedPose(Pose2d pose) {
    estimatePose = pose;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometryEstimate() {
    poseEstimator.update(drivetrain.getYawRot2d(), drivetrain.getModulePositions());
  }

  /**
   * @see edu.wpi.first.math.estimator.PoseEstimator#addVisionMeasurement(Pose2d, double)
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Reset the pose estimator location and Drivetrain odometry - NEEDS TO BE TESTED
   *
   * @param poseMeters
   */
  public void resetPoseEstimate(Pose2d poseMeters) {
    drivetrain.resetOdometry(poseMeters);
    poseEstimator.resetPosition(drivetrain.getYawRot2d(), drivetrain.getModulePositions(), drivetrain.getPose());
    
  }

  public void resetHeading(Rotation2d angle) {
    drivetrain.resetOdometry(new Pose2d(drivetrain.getPose().getTranslation(), angle));
    resetPoseEstimate(new Pose2d(estimatePose.getTranslation(), angle));
  }

  public void resetLocationEstimate(Translation2d translation) {
    resetPoseEstimate(new Pose2d(translation, new Rotation2d(0)));
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the poseEstimator. This includes
   * vision and odometry combined together.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the heading of the robot estimated by the poseEstimator. Use this in most places we would
   * use the gyro.
   *
   * @return
   */
  public Rotation2d getHeading() {
    return estimatePose.getRotation();
  }

  public Translation2d getLocation() {
    return estimatePose.getTranslation();
  }

  public Pose2d getEstimatedPose() {
    return estimatePose;
  }

  /**
   * Creates a vector of standard deviations for the states. Standard deviations of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Creates a vector of standard deviations for the local measurements. Standard deviations of
   * encoder and gyro rate measurements. Increase these numbers to trust sensor readings from
   * encoders and gyros less.
   *
   * @param theta in degrees per second
   * @param s std for all module positions in meters per sec
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
    return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
  }

  /**
   * Creates a vector of standard deviations for the vision measurements. Standard deviations of
   * global measurements from vision. Increase these numbers to trust global measurements from
   * vision less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Commnad to reset odometry of drivetrain and pose esimator to the one from vision
   * @return a command to reset the Pose Estimator and Drivetrain to the vision pose
   */
  public Command resetOdometryVisionCommand(){
    return new InstantCommand(() -> resetPoseEstimate(Vision.getInstance().visionBotPose()));
  }

  public Command tempResetOdometryCOmmand(){
    return new InstantCommand(() -> resetPoseEstimate(new Pose2d(2, 5.52, new Rotation2d(0))));
  }
  
}
