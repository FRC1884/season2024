package frc.robot;

public class RobotMap {
  public static class ElevatorMap {
    public static final int master = 4;
    public static final int slave = 5;

    public static final int limitSwitch = 0;
  }

  public static class DriveMap {
    public static final int PIGEON_ID = 9;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final int DRIVE_ID = 7;
      public static final int ROTATOR_ID = 8;
      public static final double ANGULAR_OFFSET = 0;
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRight {
      public static final int DRIVE_ID = 3;
      public static final int ROTATOR_ID = 4;
      public static final int ENCODER_ID = 12;
      public static final double ANGULAR_OFFSET = 0;
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeft {
      public static final int DRIVE_ID = 36;
      public static final int ROTATOR_ID = 37;
      public static final double ANGULAR_OFFSET = 0;
    }

    /* Back Right Module - Module 3 */
    public static final class BackRight {
      public static final int DRIVE_ID = 5;
      public static final int ROTATOR_ID = 6;
      public static final double ANGULAR_OFFSET = 0;
    }
  }

  public static class TankDriveMap {
    public static final int leftFrontMaster = 0;
    public static final int leftBackMaster = 1;
    public static final int rightFrontMaster = 2;
    public static final int rightBackMaster = 3;
  }

  public static class FlywheelMap {
    public static final int LEADER_FLYWHEEL = -1; // FIXME Set flywheel motor ID
    public static final int FOLLOWER_FLYWHEEL = -1; // FIXME Set flywheel motor ID
  }

  public static class CameraMap {
    // Rename the cameras in phtonvision dashboard to the corresponding camera name
    public static final String COMPUTER_VISION = "camscanner";
    public static final String DRIVER_CAMERA = "drivercam";
    public static final double CAMERA_HEIGHT_METRES = 0.5;
    public static final double TARGET_HEIGHT_METRES = 3.0;
    public static final double CAMERA_PITCH_RADIANS = 0.0;
  }

  public static class ControllerMap {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
  }

  public static class VisionConfig {
    public static final boolean isLimelightMode = true;
    public static final boolean isPhotonVisionMode = false;
    public static final boolean isNeuralNet = false;

    // Limelight
    public static final String POSE_LIMELIGHT = "limelight-pose";
    public static final String NN_LIMELIGHT = "limelight-nn";
    public static final int aprilTagPipeline = 1;
    public static final int noteDetectorPipeline = 2;

    public static final double POSE_LIME_X = 0.1; // Forward - Meters
    public static final double POSE_LIME_Y = 0.1; // Side
    public static final double POSE_LIME_Z = 0.1; // Up
    public static final double POSE_LIME_PITCH = 0.1; // NEED to find units
    public static final double POSE_LIME_ROLL = 0.1;
    public static final double POSE_LIME_YAW = 0.1;

    public static final double NN_LIME_X = 0.1;
    public static final double NN_LIME_Y = 0.1;
    public static final double NN_LIME_Z = 0.1;
    public static final double NN_LIME_PITCH = 0.1;
    public static final double NN_LIME_ROLL = 0.1;
    public static final double NN_LIME_YAW = 0.1;

    // Photonvision
    public static final String POSE_PHOTON_1 = "photon-cam1";
    // Translation Values (location relative to robot center)
    public static final double CAM_1_X = 0.5; // Forward: camera To Robot XMeters
    public static final double CAM_1_Y = 0.0; // Right: camera To Robot YMeters
    public static final double CAM_1_Z = 0.2; // Up: camera To Robot ZMeters

    // Rotation mounting angles (roll-pitch-yaw) in RADIANS
    public static final double CAM_1_ROLL_RADIANS = 0.0; // camera Roll Radians
    public static final double CAM_1_PITCH_RADIANS = 0.2618; // camera Pitch Radians
    public static final double CAM_1_YAW_RADIANS = 0.0; // camera Yaw Radians

    public enum POSE_STRATEGY {
      /** Choose the Pose with the lowest ambiguity. */
      LOWEST_AMBIGUITY,

      /** Choose the Pose which is closest to the camera height. */
      CLOSEST_TO_CAMERA_HEIGHT,

      /** Choose the Pose which is closest to a set Reference position. */
      CLOSEST_TO_REFERENCE_POSE,

      /** Choose the Pose which is closest to the last pose calculated */
      CLOSEST_TO_LAST_POSE,

      /** Return the average of the best target poses using ambiguity as weight. */
      AVERAGE_BEST_TARGETS,

      /**
       * Use all visible tags to compute a single pose estimate on coprocessor. This option needs to
       * be enabled on the PhotonVision web UI as well.
       */
      MULTI_TAG_PNP_ON_COPROCESSOR,
    }
  }

  public static class PoseConfig {
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevTheta = 500;
  }

  public static class PrototypeMap {
    public static final int MOTOR_ID_1 = 1;
    public static final int MOTOR_ID_2 = 8;
    public static final int MOTOR_ID_3 = 1;
    public static final int MOTOR_ID_4 = 8;
  }
}
