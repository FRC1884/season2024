package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotMap {
  public static class ElevatorMap {
    public static final int master = 4;
    public static final int slave = 5;

    public static final int limitSwitch = 0;
  }

  public enum PoseMap {
    SOURCE_BLUE(new Pose2d(0, 0, new Rotation2d(0)));

    private final Pose2d targetPose;

    PoseMap(Pose2d targetPose) {
      this.targetPose = targetPose;
    }

    public Pose2d getTargetPose() {
      return targetPose;
    }
  }

  public static class DriveMap {
    public static final int PIGEON_ID = 30;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final int DRIVE_ID = 8;
      public static final int ROTATOR_ID = 7;
      public static final double ANGULAR_OFFSET = 0;
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRight {
      public static final int DRIVE_ID = 5;
      public static final int ROTATOR_ID = 6;
      public static final double ANGULAR_OFFSET = Math.PI / 2;
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeft {
      public static final int DRIVE_ID = 2;
      public static final int ROTATOR_ID = 1;
      public static final double ANGULAR_OFFSET = -Math.PI / 2;
    }

    /* Back Right Module - Module 3 */
    public static final class BackRight {
      public static final int DRIVE_ID = 3;
      public static final int ROTATOR_ID = 4;
      public static final double ANGULAR_OFFSET = Math.PI;
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
    public static final boolean isLimelightMode = false;
    public static final boolean isPhotonVisionMode = false;
    public static final boolean isNeuralNet = false;
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.0; // Max difference between vision and odometry pose estimate
    // Field limits
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Limelight - units are meters
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
    public static final int MOTOR_ID_1 = 42;
    public static final int MOTOR_ID_2 = 8;
    public static final int MOTOR_ID_3 = 2;
    public static final int MOTOR_ID_4 = 3;

    public static final boolean MOTOR_1_ENABLED = true;
    public static final boolean MOTOR_2_ENABLED = true;
    public static final boolean MOTOR_3_ENABLED = true;
    public static final boolean MOTOR_4_ENABLED = false;

    public static final Class<?> MOTOR_1_CLASS = MOTOR_1_ENABLED
            ? CANSparkFlex.class
            : null;
    public static final Class<?> MOTOR_2_CLASS = MOTOR_2_ENABLED
            ? CANSparkMax.class
            : null;
    public static final Class<?> MOTOR_3_CLASS = MOTOR_3_ENABLED
            ? CANSparkMax.class
            : null;
    public static final Class<?> MOTOR_4_CLASS = MOTOR_4_ENABLED
            ? CANSparkMax.class
            : null;

    public static final double MOTOR_1_KP = 0.1;
    public static final double MOTOR_2_KP = 0.1;
    public static final double MOTOR_3_KP = 0.1;
    public static final double MOTOR_4_KP = 0.1;
  }

  public static class IntakeMap {
    public static final int MOTOR_ID = -1; //TODO Tune
  }
}