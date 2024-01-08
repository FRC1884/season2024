package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.core.swerve.SwerveModuleConstants;

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
      public static final int ENCODER_ID = 23;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(28);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID, OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRight {
      public static final int DRIVE_ID = 3;
      public static final int ROTATOR_ID = 4;
      public static final int ENCODER_ID = 12;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(230);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID, OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeft {
      public static final int DRIVE_ID = 36;
      public static final int ROTATOR_ID = 37;
      public static final int ENCODER_ID = 13;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(100);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID, OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRight {
      public static final int DRIVE_ID = 5;
      public static final int ROTATOR_ID = 6;
      public static final int ENCODER_ID = 11;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(342);
      public static final SwerveModuleConstants CONSTANTS =
          new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID, OFFSET);
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
}
