package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PoseConfig;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.subsystems.Drivetrain;

/** Reports our expected, desired, and actual poses to dashboards */
public class Pose extends SubsystemBase {
  // PoseConfig config;
  private PoseTelemetry telemetry;
  private Pose2d odometryPose = new Pose2d();
  private Pose2d desiredPose = new Pose2d();
  private Pose2d estimatePose = new Pose2d();

  private final SwerveDrivePoseEstimator poseEstimator;

  private static Pose instance;

  public static Pose getInstance() {
    if (instance == null) instance = new Pose();
    return instance;
  }

  private Pose() {
    // config = new PoseConfig();
    telemetry = new PoseTelemetry(this);

    // Maxswerve Version from MAXSwerve.java in core
    poseEstimator =
        new SwerveDrivePoseEstimator(
            MaxSwerveConstants.kDriveKinematics,
            Drivetrain.getInstance()
                .getYaw(), // TODO *maybe*  Make and Odometry class with easy methods for odometry
            Drivetrain.getInstance().getModulePositions(),
            new Pose2d(),
            createStateStdDevs(
                PoseConfig.kPositionStdDevX,
                PoseConfig.kPositionStdDevY,
                PoseConfig.kPositionStdDevTheta),
            createVisionMeasurementStdDevs(
                PoseConfig.kVisionStdDevX,
                PoseConfig.kVisionStdDevY,
                PoseConfig.kVisionStdDevTheta));

    // SDS Swerve Version from Swerve.java in core
    /*
    poseEstimator =
            new SwerveDrivePoseEstimator(
                    SwerveConstants.KINEMATICS,
                    Drivetrain.getInstance().getYaw(), // TODO *maybe*  Make and Odometry class with easy methods for odometry
                    Drivetrain.getInstance().getModulePositions(),
                    new Pose2d(),
                    createStateStdDevs(
                            PoseConfig.kPositionStdDevX,
                            PoseConfig.kPositionStdDevY,
                            PoseConfig.kPositionStdDevTheta),
                    createVisionMeasurementStdDevs(
                            PoseConfig.kVisionStdDevX,
                            PoseConfig.kVisionStdDevY,
                            PoseConfig.kVisionStdDevTheta));
     */
  }

  @Override
  public void periodic() {
    updateOdometryEstimate(); // Updates using wheel encoder data only
    // Updates using the vision estimate
    if (isEstimateReady(
        Vision.getInstance()
            .visionBotPose())) { // Does making so many bot pose variables impact accuracy?
      addVisionMeasurement(
          Vision.getInstance().visionBotPose(),
          Vision.getInstance().getTimestampSeconds(Vision.getInstance().getTotalLatency()));
    }
    // Robot.vision.update();

    // Update for telemetry
    setEstimatedPose(getPosition());
    setOdometryPose(Drivetrain.getInstance().getPose());

    // telemetry.updatePoseOnField("VisionPose", Robot.vision.botPose);
    telemetry.updatePoseOnField("OdometryPose", odometryPose);
    telemetry.updatePoseOnField(
        "EstimatedPose", estimatePose); // Need to uncomment and fix to work here.
  }

  // /**
  //  * @return true if estimated pose is on the chargestation by using the field-space
  // chargestation
  //  *     dimensions
  //  */
  // public boolean isOnChargeStation() {
  //     return ((getBestPose().getX() > 2.9 && getBestPose().getX() < 4.8)
  //             && (getBestPose().getY() > 1.54 && getBestPose().getY() < 3.99));
  // }

  // /**
  //  * Returns the most accurate pose. If we are not confident that vision is accurate,
  //  * estimatedPose is considered to be most accurate.
  //  *
  //  * @return vision pose or estimated pose
  //  */
  // public Pose2d getBestPose() {
  //     if (Robot.vision.visionAccurate()) {
  //         return Robot.vision.botPose;
  //     } else {
  //         return estimatePose;
  //     }
  // }

  /**
   * Helper method for comparing vision pose against odometry pose. Does not account for difference
   * in rotation. Will return false vision if it sees no targets or if the vision estimated pose is
   * too far from the odometry estimate
   *
   * @return whether or not pose should be added to estimate or not
   */
  public boolean isEstimateReady(Pose2d pose) {
    /* Disregard Vision if there are no targets in view */
    if (!Vision.getInstance()
        .visionAccurate()) { // visionAccurate method sees if Apriltags present in Vision.java
      return false;
    }

    /* Disregard Vision if odometry has not been set to vision pose yet in teleopInit*/
    // Pose2d odometryPose = Robot.core.TalonSwerve.getPoseMeters();

    if (odometryPose.getX() <= 0.3
        && odometryPose.getY() <= 0.3
        && odometryPose.getRotation().getDegrees() <= 1) {
      return false;
    }
    return (Math.abs(pose.getX() - odometryPose.getX()) <= 1)
        && (Math.abs(pose.getY() - odometryPose.getY())
            <= 1); // this can be tuned to find a threshold that helps us remove jumping vision
    // poses
  }

  /** Sets the Odometry Pose to the given pose */
  public void setOdometryPose(Pose2d pose) {
    odometryPose = pose;
  }

  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Sets the desired pose of the robot */
  public void setDesiredPose(Pose2d pose) {
    desiredPose = pose;
  }

  /** Sets the estimated pose to the given pose */
  public void setEstimatedPose(Pose2d pose) {
    estimatePose = pose;
  }

  /** Moved from Vision class --> LSAO: THIS SHOULD BE A POSE CLASS POWER, NOT A VISION POWER */
  /*
  public void resetEstimatedPose() {
      Robot.pose.resetPoseEstimate(botPose);
  }
  */

  /** Updates the field relative position of the robot. */
  // Lsao Jan 11: Changed naming to match Drivetrain file - methods have same name for both
  // MaxSwerve.java and Swerve.java
  public void updateOdometryEstimate() {
    poseEstimator.update(
        Drivetrain.getInstance().getYaw(), Drivetrain.getInstance().getModulePositions());
  }

  /**
   * Add a vision measurement to the PoseEstimator. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime}
   *     then you must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
   *     timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
   *     Timer.getFPGATimestamp as your time source or sync the epochs.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * reset the pose estimator - Fix these during session
   *
   * @param poseMeters
   */
  /*
  public void resetPoseEstimate(Pose2d poseMeters) {
      Drivetrain.resetOdometry(poseMeters);
      poseEstimator.resetPosition(
              Robot.swerve.getRotation(), Robot.swerve.getPositions(), poseMeters);
  }


  public void resetHeading(Rotation2d angle) {
      Robot.swerve.odometry.resetHeading(angle);
      resetPoseEstimate(new Pose2d(estimatePose.getTranslation(), angle));
  }

  public void resetLocationEstimate(Translation2d translation) {
      resetPoseEstimate(new Pose2d(translation, estimatePose.getRotation()));
  }
  */

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
  public Vector<N5> createLocalMeasurementStdDevs(double theta, double p) {
    return VecBuilder.fill(Units.degreesToRadians(theta), p, p, p, p);
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

  /*
  public static Command resetHeading(Rotation2d heading) {
      return new InstantCommand(() -> Robot.pose.resetHeading(heading));
  }

  public static Command resetHeading(double headingDeg) {
      return new InstantCommand(
              () -> Robot.pose.resetHeading(Rotation2d.fromDegrees(headingDeg)));
  }
  */

}
