package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Config.RobotType;
import frc.robot.util.FlywheelLookupTable;

@SuppressWarnings("ALL")
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

        public static final GyroType GYRO_TYPE = GyroType.NAVX;

        
        public static final double GYRO_OFFSET_RADIANS = switch (Config.ROBOT_TYPE) {
            case DEV -> 0;
            case COMP -> Math.PI / 2;
        };

        public static final int PIGEON_ID = 30;

        public static final double SLOW_MODE_TRANSLATE_MULTIPLIER = 0.3;

        public static final double SLOW_MODE_ROTATION_MUTLIPLIER = 0.2;

        public static double SPEAKER_ALIGN_OFFSET = 0;

        public static boolean IS_SLOWMODE_ENABLED = false;

        public static double SPEAKER_ALIGN_TOLERANCE = 1.5;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft {
            public static final int DRIVE_ID = switch (Config.ROBOT_TYPE) {
              case DEV -> 15;
              case COMP -> 2;
            };

            public static final int ROTATOR_ID = switch (Config.ROBOT_TYPE) {
              case DEV -> 14;
              case COMP -> 1;
            };

            public static final double ANGULAR_OFFSET = -Math.PI / 2;
            ;
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight {
          public static final int DRIVE_ID = switch (Config.ROBOT_TYPE) {
            case DEV -> 11;
            case COMP -> 6;
          };

          public static final int ROTATOR_ID = switch (Config.ROBOT_TYPE) {
            case DEV -> 10;
            case COMP -> 5;
          };

            public static final double ANGULAR_OFFSET = 0;
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeft {
          public static final int DRIVE_ID = switch (Config.ROBOT_TYPE) {
            case DEV -> 17;
            case COMP -> 4;
          };

          public static final int ROTATOR_ID = switch (Config.ROBOT_TYPE) {
            case DEV -> 16;
            case COMP -> 3;
          };

            public static final double ANGULAR_OFFSET = Math.PI;
        }

        /* Back Right Module - Module 3 */
        public static final class BackRight {
            public static final int DRIVE_ID = switch (Config.ROBOT_TYPE) {
              case DEV -> 13;
              case COMP -> 8;
            };

            public static final int ROTATOR_ID = switch (Config.ROBOT_TYPE) {
              case DEV -> 12;
              case COMP -> 7;
            };

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
        public static final double X_CENTERLINE_LIMIT_RED = 8.275 + 0.0;
        public static final double X_CENTERLINE_LIMIT_BLUE = 8.275 - 0.0;
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
        public static final boolean IS_LIMELIGHT_APRILTAG_MODE = false;
        public static final boolean IS_PHOTON_VISION_ENABLED = true;
        public static final boolean IS_PHOTON_TWO_ENABLED = true;
        public static final boolean IS_PHOTON_THREE_ENABLED = true;
        public static final boolean IS_NEURAL_NET_LIMELIGHT = true;
        public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose
        
        public enum CAMERA_TYPE {
          OV2311,
          OV9281, 
          TELEPHOTO_OV9281,
        };

        // estimate
        public static final int MOVING_AVG_TAPS = 5; //TODO: Change to optimum value

        public static final String DRIVER_CAM_STREAM = "http://drivercam.local:1182/stream.mjpg";

        // Field limits
        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;
        public static final double VISION_X_MIN_CUTOFF = 3.0;
        public static final double VISION_X_MAX_CUTOFF = 13.5; 

        //Noisy Distance Constanst
        public static final double OV2311_NOISY_DISTANCE_METERS = 3.6;
        public static final double OV9281_NOISY_DISTANCE_METERS = 3.5;
        public static final double TELEPHOTO_NOISY_DISTANCE_METERS = 5.0;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double TAG_PRESENCE_WEIGHT = 10;
        public static final double DISTANCE_WEIGHT = 7;

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
        public static final double NN_LIME_X = switch (Config.ROBOT_TYPE) {
            case DEV -> -0.3969; // +X is forward on the robot
            case COMP -> -0.311;
        };

        public static final double NN_LIME_Y = switch (Config.ROBOT_TYPE) {
            case DEV -> -0.19685; // +Y is the the left of the robot
            case COMP -> -0.276;
        };
        public static final double NN_LIME_Z = switch (Config.ROBOT_TYPE) {
            case DEV -> 0.4325; // +Z is up
            case COMP -> 0.453 + 0.066;
        };
        public static final double NN_LIME_PITCH = switch (Config.ROBOT_TYPE) {
            case DEV -> -0.349; 
            case COMP -> -0.349;
        };
        public static final double NN_LIME_ROLL = switch (Config.ROBOT_TYPE) {
            case DEV -> -0.0; 
            case COMP -> 0.0;
        };
        public static final double NN_LIME_YAW = Math.PI;

        public static final Transform2d NN_ROBOT_TO_LIME_2D = new Transform2d(NN_LIME_X, NN_LIME_Y, new Rotation2d(NN_LIME_YAW));

        public static final Transform2d NN_LIME_TO_ROBOT_2D = new Transform2d(-NN_LIME_X, -NN_LIME_Y, new Rotation2d(-NN_LIME_YAW));

        // Photonvision
        public static final double  POSE_AMBIGUITY_CUTOFF = 0.6;

        public static final String POSE_PHOTON_1 = "photoncam-1";
        public static final CAMERA_TYPE CAM_1_TYPE = CAMERA_TYPE.OV2311;

        // Translation Values (location relative to robot center)
        public static final double CAM_1_X = Config.ROBOT_TYPE == RobotType.DEV ? 0.30726 : 0.306; // Forward: camera To Robot XMeters
        public static final double CAM_1_Y = Config.ROBOT_TYPE == RobotType.DEV ? - 0.17780 - 0.0127 : 0.038; // Left: camera To Robot YMeters
        public static final double CAM_1_Z =Config.ROBOT_TYPE == RobotType.DEV ?  0.156 : 0.091 + 0.066; // Up: camera To Robot ZMeters

        // Rotation mounting angles (roll-pitch-yaw) in RADIANS
        public static final double CAM_1_ROLL_RADIANS = Config.ROBOT_TYPE == RobotType.DEV ? Math.PI : 0; // camera Roll Radians
        public static final double CAM_1_PITCH_RADIANS = 0.4887; // camera Pitch Radians
        public static final double CAM_1_YAW_RADIANS = 0.0; // camera Yaw Radians, +CCW

        public static final Transform3d PHOTON_1_ROBOT_TO_CAM = new Transform3d(CAM_1_X, CAM_1_Y, CAM_1_Z, new Rotation3d(CAM_1_ROLL_RADIANS, CAM_1_PITCH_RADIANS, CAM_1_YAW_RADIANS));
        public static final Transform3d PHOTON_1_CAM_TO_ROBOT = new Transform3d(-CAM_1_X, -CAM_1_Y, -CAM_1_Z, new Rotation3d(-CAM_1_ROLL_RADIANS, -CAM_1_PITCH_RADIANS, -CAM_1_YAW_RADIANS));

        // Photonvision
        public static final String POSE_PHOTON_2 = "photoncam-2";
        public static final CAMERA_TYPE CAM_2_TYPE = CAMERA_TYPE.OV2311;
        // Translation Values (location relative to robot center)
        public static final double CAM_2_X = Config.ROBOT_TYPE == RobotType.DEV ? -0.28928 : -0.306; // Forward: camera To Robot XMeters
        public static final double CAM_2_Y = Config.ROBOT_TYPE == RobotType.DEV ?  0.23315 : 0; // Right: camera To Robot YMeters
        public static final double CAM_2_Z = Config.ROBOT_TYPE == RobotType.DEV ? 0.3173 : 0.117 + 0.066; // Up: camera To Robot ZMeters

        // Rotation mounting angles (roll-pitch-yaw) in RADIANS
        public static final double CAM_2_ROLL_RADIANS = Config.ROBOT_TYPE == RobotType.DEV ? Math.PI : 0; // camera Roll Radians
        public static final double CAM_2_PITCH_RADIANS = 0.4887; // camera Pitch Radians
        public static final double CAM_2_YAW_RADIANS = Math.PI; // camera Yaw Radians


        public static final Transform3d PHOTON_2_ROBOT_TO_CAM = new Transform3d(CAM_2_X, CAM_2_Y, CAM_2_Z, new Rotation3d(CAM_2_ROLL_RADIANS, CAM_2_PITCH_RADIANS, CAM_2_YAW_RADIANS));
        public static final Transform3d PHOTON_2_CAM_TO_ROBOT = new Transform3d(-CAM_2_X, -CAM_2_Y, -CAM_2_Z, new Rotation3d(-CAM_2_ROLL_RADIANS, -CAM_2_PITCH_RADIANS, -CAM_2_YAW_RADIANS));

        // Photonvision
        public static final String POSE_PHOTON_3 = "laser-cam";
        public static final CAMERA_TYPE CAM_3_TYPE = CAMERA_TYPE.TELEPHOTO_OV9281;
        // Translation Values (location relative to robot center)
        public static final double CAM_3_X = Config.ROBOT_TYPE == RobotType.DEV ? 0.30726 : 0.319; // Forward: camera To Robot XMeters
        public static final double CAM_3_Y = Config.ROBOT_TYPE == RobotType.DEV ? - 0.17780 + 0.0254 : -0.038; // Left: camera To Robot YMeters
        public static final double CAM_3_Z = Config.ROBOT_TYPE == RobotType.DEV ? 0.156 : 0.117 + 0.066; // Up: camera To Robot ZMeters

        // Rotation mounting angles (roll-pitch-yaw) in RADIANS
        public static final double CAM_3_ROLL_RADIANS = 0; // camera Roll Radians
        public static final double CAM_3_PITCH_RADIANS = 0.20944; // camera Pitch Radians
        public static final double CAM_3_YAW_RADIANS = 0.0; // camera Yaw Radians, +CCW

        public static final Transform3d PHOTON_3_ROBOT_TO_CAM = new Transform3d(CAM_3_X, CAM_3_Y, CAM_3_Z, new Rotation3d(CAM_3_ROLL_RADIANS, CAM_3_PITCH_RADIANS, CAM_3_YAW_RADIANS));
        public static final Transform3d PHOTON_3_CAM_TO_ROBOT = new Transform3d(-CAM_3_X, -CAM_3_Y, -CAM_3_Z, new Rotation3d(-CAM_3_ROLL_RADIANS, -CAM_3_PITCH_RADIANS, -CAM_3_YAW_RADIANS));
        
    }
    public static class PoseConfig {
        // Increase these numbers to trust your model's state estimates less.
        public static final double kPositionStdDevX = 0.1;
        public static final double kPositionStdDevY = 0.1;
        public static final double kPositionStdDevTheta = 10;

        // Increase these numbers to trust global measurements from vision less.
        public static final double kVisionStdDevX = 2;
        public static final double kVisionStdDevY = 2;
        public static final double kVisionStdDevTheta = 1 * Math.PI;
        
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
                kVisionStdDevX, // x
                kVisionStdDevY, // y
                kVisionStdDevTheta // theta
            );
;
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
        public static final int INTAKE_ID = switch (Config.ROBOT_TYPE) { 
          case DEV -> 20;
          case COMP -> 23;
        };

        public static final double INTAKE_FORWARD_SPEED = 1;

        public static final double INTAKE_REVERSE_SPEED = -0.6;
    }

    public static class ShooterMap {

        public static final int TOP_SHOOTER = switch (Config.ROBOT_TYPE) {
          case DEV -> 30;
          case COMP -> 33;
        };
      
        public static final int BOTTOM_SHOOTER = switch (Config.ROBOT_TYPE) {
          case DEV -> 31;
          case COMP -> 34;
        };
        
        public static final double FLYWHEEL_RADIUS = 0.0508;
        public static final double FLYWHEEL_VELOCITY_TOLERANCE = 50;
        public static final double AMP_SPEED_FOLLOW = 600;
        public static final double AMP_SPEED_LEAD = switch (Config.ROBOT_TYPE) {
            case COMP -> 300;
            case DEV -> 600;
        };
        
        public static final double TRAP_SPEED = 3000;
        public static final double SHOOTER_INTAKE_SPEED = -1000;

        public static final PIDConstants FLYWHEEL_PID = new PIDConstants(0.00036, 0, 0.015);
        public static final double FLYWHEEL_FF = 0.00015;
        public static final double FLYWHEEL_RAMP_RATE = 0.5;

        public static final double[][] SINGLE_ACTUATOR_VALUES = {
                {1, 2600, -75},
                {1.45, 2600, -58.2},
                {1.95, 2600, -40.2},
                {2.47, 3000, -28.2},
                //  { 2.49, 3700, -52- 12},
                //  { 2.90, 3800, -52- 12},
                //  { 2.96, 4000, -30 - 12},
                {3.40, 5000, -19},
                {3.7, 6000, -16},
                // {3.95, 6000, -12},
                {4.03, 6000, -14},
                {4.3, 6000, -10.5},
                {4.6, 6000, -9}
        };

        public static final double[][] DUAL_ACTUATOR_VALUES = {
                {0.912, 5600, 0.9},
                {1.471, 5600, 0.77},
                {1.979, 5600, 0.70},
                {2.492, 5600, 0.63},
                {2.96, 5600, 0.575},
                {3.57, 5600, 0.518},
                {3.87, 5900, 0.495},
                {4.43, 6500, 0.482},
                {5.02, 6600, 0.461},
                {5.41, 6550, 0.452},
                {8.00, 6550, 0.452}
        };

        public static final double[][] SPEAKER_SHOT_VALUES = switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
            case SINGLE_ACTUATOR -> SINGLE_ACTUATOR_VALUES;
            case DUAL_ACTUATOR, NONE -> DUAL_ACTUATOR_VALUES;
        };

        public static final double[][] FERRY_SHOT_VALUES = {
                {0, 0, 0}
        };

        public static final FlywheelLookupTable SPEAKER_LOOKUP_TABLE = new FlywheelLookupTable(SPEAKER_SHOT_VALUES);

        public static final FlywheelLookupTable FERRY_LOOKUP_TABLE = new FlywheelLookupTable(FERRY_SHOT_VALUES);
    }

    public static class FeederMap {
        public static final int FEEDER =  switch (Config.ROBOT_TYPE) {
          case DEV -> 21;
          case COMP -> 24;
        };

        public static final PIDConstants FEEDER_PID = new PIDConstants(0.00036, 0, 0.015);
        public static final double FEEDER_FF = 0.00015;
        public static final double FEEDER_RAMP_RATE = 0.5;
        public static final double FEEDER_RPM = 3500 * 4; //multiplied by four to account for gear ratio
        public static final double FEEDER_RPM_SLOW = 1000 * 4;
        
        public static final int LOWER_BEAMBREAK = 0;
        public static final int UPPER_BEAMBREAK = 1;
    }

    public static class ClimberMap {
        public static final int LEADER_ID = 21;
        public static final int FOLLOWER_ID = 22;

        public static final int LIMIT_SWITCH = 5;

        public static final double P = 0.0003;
        public static final double I = 0.0000008;
        public static final double D = 0.0000006;

        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 10);

        public static final int SERVO_ID_1 = 1;
        public static final int SERVO_ID_2 = 2;

        public static final double TOP_VALUE = 1;
        public static final double LOCKED_VALUE = 0;
    }

    public static class PivotMap {
        public static final double POSITION_TOLERANCE =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> .1;
                    default -> 0.005;
                };

        public static final double VELOCITY_TOLERANCE =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> .5;
                    default -> 0.031;
                };

        public static final double UPPER_SETPOINT_LIMIT =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> 0;
                    default -> 2.01;
                };

        public static final double LOWER_SETPOINT_LIMIT =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> 0;
                    default -> 0.267;
                };

        public static final double PIVOT_AMP_ANGLE =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> -325 * 15 / 25;
                    default -> 1.85;
                };

        public static final double PIVOT_TRAP_ANGLE =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> -150 * 15 / 25;
                    default -> 0;
                };

        public static final double PIVOT_INTAKE_ANGLE =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> -100 * 15 / 25;
                    default -> 1.7;
                };

        public static final double PIVOT_RESTING_ANGLE =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> -1;
                    default -> 0.45;
                };

        public static final double kP =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> 1.5;
                    default -> 25;
                };

        public static final double kI =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> 3;
                    default -> 5;
                };

        public static final double kD =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> 0.075;
                    default -> 0;
                };

        public static final double kIZone =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> 0.5;
                    default -> 0.05;
                };

        public static final double kG = switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
            case SINGLE_ACTUATOR -> 0;
            default -> 0.62;
        };

        public static final double kV = switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
            case SINGLE_ACTUATOR -> 0;
            default -> 0.55;
        };

        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS =
                switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
                    case SINGLE_ACTUATOR -> new TrapezoidProfile.Constraints(200, 100);
                    default -> new TrapezoidProfile.Constraints(200, 1);
                };
    }

    public static class SingleActuatorPivotMap {
        public static final int PIVOT_ID = 22;
    }

    public static class DoubleActuatorPivotMap {
        // hardstop angle in degrees
        public static final double HARDSTOP_ANGLE = 13.2;
        public static final double HARDSTOP_ENCODER = 0.056;
        public static final int PIVOT_ID_LEADER = 32;
        public static final int PIVOT_ID_FOLLOWER = 31;
        public static final int ENCODER_PORT = 2;
        public static double ENCODER_SCALE = 2 * Math.PI;
        public static double ENCODER_OFFSET = (HARDSTOP_ANGLE / 360) - HARDSTOP_ENCODER ;
    }

    public static class LEDMap {
        public static final int BLINKIN_PWM_PORT = 0;
        public static final boolean BLINKIN_PWM = true;
        public static final boolean BLINKIN_ON_SPARK = false;
        public static final int NUMBER_LEDS = switch (Config.ROBOT_TYPE) {
            case DEV -> 42;
            case COMP -> 17*4 + 21 * 2;
        };
    }
}
