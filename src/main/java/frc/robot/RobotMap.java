package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.FlywheelLookupTable;

public class RobotMap {
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

    public enum GyroType {
      PIGEON, NAVX
    }

    public static final GyroType GYRO_TYPE = GyroType.PIGEON;

    public static final int PIGEON_ID = 30;

    public static final double SLOW_MODE_TRANSLATE_MULTIPLIER = 0.3;

    public static final double SLOW_MODE_ROTATION_MUTLIPLIER = 0.2;

    public static double SPEAKER_ALIGN_OFFSET = 0;

    public static boolean IS_SLOWMODE_ENABLED = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final int DRIVE_ID = 1;
      public static final int ROTATOR_ID = 2;
      public static final double ANGULAR_OFFSET = -Math.PI / 2;;
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRight {
      public static final int DRIVE_ID = 3;
      public static final int ROTATOR_ID = 4;
      public static final double ANGULAR_OFFSET = 0;
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeft {
      public static final int DRIVE_ID = 5;
      public static final int ROTATOR_ID = 6;
      public static final double ANGULAR_OFFSET = Math.PI;
    }

    /* Back Right Module - Module 3 */
    public static final class BackRight {
      public static final int DRIVE_ID = 7;
      public static final int ROTATOR_ID = 8;
      public static final double ANGULAR_OFFSET = Math.PI / 2;
    }

  }

  // TODO rename to path swerve constants.
  public static final class SwervePathFollowConstants {
    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ACCELERATION = 4;
    public static final double MAX_ANG_VELOCITY = 1;
    public static final double MAX_ANG_ACCELERATION = 1;

  }

  public static final class Coordinates {
    public static final Pose2d BLUE_SPEAKER = new Pose2d(0.22, 5.55, new Rotation2d(Math.PI));
    public static final Pose2d BLUE_AMP = new Pose2d(1.79, 7.60, new Rotation2d(Math.PI / 2));
    public static final Pose2d BLUE_SOURCE = new Pose2d(15.3, 1.11, Rotation2d.fromDegrees(-55));
    public static final Pose2d RED_SPEAKER = new Pose2d(16.54175 - 0.22, 5.55, new Rotation2d(Math.PI));
    public static final Pose2d RED_AMP = new Pose2d(14.68, 7.52, new Rotation2d(Math.PI / 2));
    public static final Pose2d RED_SOURCE = new Pose2d(1.14, 1.00, Rotation2d.fromDegrees(-120));
    public static final Pose2d RED_STAGE = new Pose2d(13, 2, Rotation2d.fromDegrees(120));
  }

  public static class TankDriveMap {
    public static final int leftFrontMaster = 0;
    public static final int leftBackMaster = 1;
    public static final int rightFrontMaster = 2;
    public static final int rightBackMaster = 3;
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
    public static final boolean DRIVER_CAMERA_ACTIVE = false;
    public static final boolean VISION_OVERRIDE_ENABLED = false;
    public static final boolean IS_LIMELIGHT_MODE = false;
    public static final boolean IS_PHOTON_VISION_MODE = true;// TODO: Change to false
    public static final boolean IS_NEURAL_NET = true;
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose
                                                                  // estimate

    public static final String DRIVER_CAM_STREAM = "http://drivercam.local:1182/stream.mjpg";    

    // Field limits
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
    public static final double VISION_X_MIN_CUTOFF = 3.0;
    public static final double VISION_X_MAX_CUTOFF = 13.5;

    // Limelight - units are meters
    public static final String POSE_LIMELIGHT = "limelight-pose";
    public static final String NN_LIMELIGHT = "limelight-nn";
    public static final int APRILTAG_PIPELINE = 1;
    public static final int NOTE_DETECTOR_PIPELINE = 2;

    public static final double POSE_LIME_X = 0.322; // Forward - Meters
    public static final double POSE_LIME_Y = -0.274; // Side - Right is positive on the limelight
    public static final double POSE_LIME_Z = 0.21; // Up
    public static final double POSE_LIME_PITCH = 30; // NEED to find units - degrees for now
    public static final double POSE_LIME_ROLL = 0.0;
    public static final double POSE_LIME_YAW = 0.0;

    //THESE ARE ALL ROBOT RELATIVE (CENTER OF THE ROBOT IS THE ORIGIN)
    public static final double NN_LIME_X = -0.308; // +X is forward on the robot
    public static final double NN_LIME_Y = -0.300; // +Y is the the left of the robot
    public static final double NN_LIME_Z = 0.497; // +Z is up
    public static final double NN_LIME_PITCH = -0.349;
    public static final double NN_LIME_ROLL = 0.0;
    public static final double NN_LIME_YAW = Math.PI;

    public static final Transform2d NN_ROBOT_TO_LIME_2D = new Transform2d(NN_LIME_X, NN_LIME_Y, new Rotation2d(NN_LIME_YAW));

    public static final Transform2d NN_LIME_TO_ROBOT_2D = new Transform2d(-NN_LIME_X, -NN_LIME_Y, new Rotation2d(-NN_LIME_YAW));

    // Photonvision
    public static final double POSE_AMBIGUITY_CUTOFF = 0.2;

    public static final String POSE_PHOTON_1 = "photoncam-1";
    // Translation Values (location relative to robot center)
    public static final double CAM_1_X = 0.3079; // Forward: camera To Robot XMeters
    public static final double CAM_1_Y = -0.0762; // Left: camera To Robot YMeters
    public static final double CAM_1_Z = 0.156; // Up: camera To Robot ZMeters

    // Rotation mounting angles (roll-pitch-yaw) in RADIANS
    public static final double CAM_1_ROLL_RADIANS = 0.0; // camera Roll Radians
    public static final double CAM_1_PITCH_RADIANS = 0.436; // camera Pitch Radians
    public static final double CAM_1_YAW_RADIANS = 0.0; // camera Yaw Radians, +CCW

    public static final Transform3d PHOTON_1_ROBOT_TO_CAM = new Transform3d(CAM_1_X, CAM_1_Y, CAM_1_Z, new Rotation3d(CAM_1_ROLL_RADIANS, CAM_1_PITCH_RADIANS, CAM_1_YAW_RADIANS));
    public static final Transform3d PHOTON_1_CAM_TO_ROBOT = new Transform3d(-CAM_1_X, -CAM_1_Y, -CAM_1_Z, new Rotation3d(-CAM_1_ROLL_RADIANS, -CAM_1_PITCH_RADIANS, -CAM_1_YAW_RADIANS)); 

    // Photonvision
    public static final String POSE_PHOTON_2 = "photoncam-2";
    // Translation Values (location relative to robot center)
    public static final double CAM_2_X = 0.5; // Forward: camera To Robot XMeters
    public static final double CAM_2_Y = 0.0; // Right: camera To Robot YMeters
    public static final double CAM_2_Z = 0.2; // Up: camera To Robot ZMeters

    // Rotation mounting angles (roll-pitch-yaw) in RADIANS
    public static final double CAM_2_ROLL_RADIANS = 0.0; // camera Roll Radians
    public static final double CAM_2_PITCH_RADIANS = 0.2618; // camera Pitch Radians
    public static final double CAM_2_YAW_RADIANS = 0.0; // camera Yaw Radians
  }

  public static class PoseConfig {
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 2.5;
    public static final double kVisionStdDevY = 2.5;
    public static final double kVisionStdDevTheta = 500;
  }

  public static class PrototypeMap {
    public static final boolean LIVE_WINDOW_ENABLED = true;

    public static final int MOTOR_ID_1 = 14;
    public static final int MOTOR_ID_2 = 8;
    public static final int MOTOR_ID_3 = 50;
    public static final int MOTOR_ID_4 = 3;

    public static final double MOTOR_1_KP = 0.1;
    public static final double MOTOR_1_KI = 0.0;
    public static final double MOTOR_1_KD = 0.0;

    public static final double WHEEL_RADIUS = 0.0508;
  }
  public static class IntakeMap {
    public static final int INTAKE_ID = 9;

    public static final double INTAKE_FORWARD_SPEED = 1;

    public static final double INTAKE_REVERSE_SPEED = -0.6;
  }

  public static class ShooterMap {

    public static final int TOP_SHOOTER = 11;
    public static final int BOTTOM_SHOOTER = 14;
    public static final double FLYWHEEL_RADIUS = 0.0508;
    public static final double FLYWHEEL_VELOCITY_TOLERANCE = 50;
    public static final double AMP_SPEED = 600;
    public static final double TRAP_SPEED = 3000;

    public static final PIDConstants FLYWHEEL_PID = new PIDConstants(0.00036, 0, 0.015);
    public static final double FLYWHEEL_FF = 0.00015;
    public static final double FLYWHEEL_RAMP_RATE = 0.5;

    public static final double[][] SPEAKER_SHOT_VALUES = {
      { 1, 2600,-115-10},
      { 1.45, 2600, -87-10},
      { 1.95, 2600, -67},
      { 2.47, 3000, -52+5},
    //  { 2.49, 3700, -52- 12},
    //  { 2.90, 3800, -52- 12},
    //  { 2.96, 4000, -30 - 12},
      { 3.48, 4000, -19 +5},
      { 3.95, 4050, -10 +5},
      { 4.43, 4200, -4}
    };
      
    public static final double [][] FERRY_SHOT_VALUES = {
      {0, 0, 0}
    };

    public static final FlywheelLookupTable SPEAKER_LOOKUP_TABLE = new FlywheelLookupTable(SPEAKER_SHOT_VALUES);

    public static final FlywheelLookupTable FERRY_LOOKUP_TABLE = new FlywheelLookupTable(FERRY_SHOT_VALUES);
  }

  public static class FeederMap {
    public static final int FEEDER = 10;
    public static final PIDConstants FEEDER_PID = new PIDConstants(0.00036, 0, 0.015);
    public static final double FEEDER_FF = 0.00015;
    public static final double FEEDER_RAMP_RATE = 0.5;
    public static final double FEEDER_RPM = 3500 *4; //multiplied by four to account for gear ratio
    public static final double FEEDER_RPM_SLOW = 1000 * 4;
    public static final int BEAMBREAK = 0;
  }

  public static class ClimberMap {
    public static final int MASTER_ID = 16;
    public static final int SLAVE_ID = 17;

    public static final int LIMIT_SWITCH = 5;

    public static final double P = 0.0003;
    public static final double I = 0.0000008;
    public static final double D = 0.0000006;

    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 10);

    public enum Setpoints {
      DOWN(0),
      UP(1000);

      private double ticks;

      Setpoints(double ticks) {
        this.ticks = ticks;
      }

      public double getTicks() {
        return ticks;
      }
    }
  }

  public static class PivotMap {
    public static final int PIVOT_ID = 13;
    public static final double POSITION_TOLERANCE = 1;
    public static final double VELOCITY_TOLERANCE = .5;
    public static final double DT = 0.2;
    public static final double UPPER_SETPOINT_LIMIT = -350 * 15/25;
    public static final double LOWER_SETPOINT_LIMIT = 0;
    public static final double PIVOT_AMP_ANGLE = -325 * 15/25;
    public static final double PIVOT_TRAP_ANGLE = -150 * 15/25;

    // TODO: Tune these values
    public static final double kP = 1.5;
    public static final double kI = 3;
    public static final double kD = 0.075;

    public static final double kIZone = 0.5;

    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(200, 100);
  }

  public static class LEDMap {
    public static final int BLINKIN_PWM_PORT = 6;
    public static final boolean BLINKIN_PWM = true;
    public static final boolean BLINKIN_ON_SPARK = false;
    public static final int NUMBER_LEDS = 22;
  }
}
