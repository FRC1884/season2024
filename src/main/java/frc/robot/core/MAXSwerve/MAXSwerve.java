package frc.robot.core.MAXSwerve;

import static frc.robot.core.TalonSwerve.SwerveConstants.KINEMATICS;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.core.MAXSwerve.MaxSwerveConstants.*;
import frc.robot.core.TalonSwerve.SwerveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class MAXSwerve extends SubsystemBase {

  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(MaxSwerveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(MaxSwerveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private MAXSwerveModule fl, fr, bl, br;
  private Pigeon2 gyro;
  private ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  GenericEntry distanceEntry = tab.add("Distance to target", 0).getEntry();

  SwerveDriveOdometry odometry;

  public MAXSwerve(
      int pigeon_id,
      MAXSwerveModule fl,
      MAXSwerveModule fr,
      MAXSwerveModule bl,
      MAXSwerveModule br) {
    this.gyro = new Pigeon2(pigeon_id);
    // gyro.getConfigurator().DefaultTimeoutSeconds = 50;
    zeroGyro();
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;

    odometry =
        new SwerveDriveOdometry(
            MaxSwerveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[] {
              fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
            });
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
          fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
        });
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = fl.getPosition();
    positions[1] = fr.getPosition();
    positions[2] = bl.getPosition();
    positions[3] = br.getPosition();
    return positions;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
          fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(MaxSwerveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = MAXSwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir =
            MAXSwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = MAXSwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir =
            MAXSwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * MaxSwerveConstants.kMaxSpeedMetersPerSecond;

    double ySpeedDelivered = ySpeedCommanded * MaxSwerveConstants.kMaxSpeedMetersPerSecond;

    double rotDelivered = currentRotation * MaxSwerveConstants.kMaxAngularSpeed;
    var swerveModuleStates =
        MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(gyro.getYaw()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);

    fl.setDesiredState(swerveModuleStates[0]);
    fr.setDesiredState(swerveModuleStates[1]);
    bl.setDesiredState(swerveModuleStates[2]);
    br.setDesiredState(swerveModuleStates[3]);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
    fl.setDesiredState(swerveModuleStates[0]);
    fr.setDesiredState(swerveModuleStates[1]);
    bl.setDesiredState(swerveModuleStates[2]);
    br.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds speeds =
        KINEMATICS.toChassisSpeeds(fl.getState(), fr.getState(), bl.getState(), br.getState());
    return speeds;
  }

  public Command driveCommand(
      Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed) {
    return new RepeatCommand(
        new RunCommand(
            () -> this.drive(xSpeed.get(), ySpeed.get(), rotSpeed.get(), true, true), this));
  }

  public BooleanSupplier getShouldFlip() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return followPathCommand(path);
  }

  public Command followPathCommand(PathPlannerPath pathName) {

    return new FollowPathHolonomic(
        pathName,
        this::getPose, // Robot pose supplier
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveWithChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        getShouldFlip(),
        this // Reference to this subsystem to set requirements
        );
  }

  public Command followAprilTagCommand() {
    return new RepeatCommand(
        new RunCommand(
            () ->
                this.followPathCommand(
                    new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                            // Vision.getInstance().getRobotPose2d_TargetSpace(),
                            new Pose2d(1.0, 0.0, new Rotation2d())), // Need to make this better
                        null,
                        null) // null vaules because these are to be obtained from vision when that
                    // is finished
                    ),
            this));
  }

  public Command navigate(Pose2d targetPose) {
    return new RunCommand(
        () ->
            this.followPathCommand(
                new PathPlannerPath(
                    PathPlannerPath.bezierFromPoses(getPose(), targetPose),
                    null,
                    null) // null vaules because these are to be obtained from vision when that is
                // finished
                ),
        this);
  }

  public Command goSpeakerOrSource(boolean hasNote) {
    if (hasNote) {
      return navigate(getPose());
    } else {
      return navigate(getPose());
    }
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
    fl.setDesiredState(desiredStates[0]);
    fr.setDesiredState(desiredStates[1]);
    bl.setDesiredState(desiredStates[2]);
    br.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    fl.resetEncoders();
    fr.resetEncoders();
    bl.resetEncoders();
    br.resetEncoders();
  }

  /** Zeros the heading of the robot */
  public void ZeroHeading() {
    gyro.setYaw(0);
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw()).getDegrees();
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
}

//   /**
//    * Returns the turn rate of the robot.
//    *
//    * @return The turn rate of the robot, in degrees per second
//    */
//   public double getTurnRate() {
//     return gyro.getRate() * (MaxSwerveConstants.kGyroReversed ? -1.0 : 1.0);
//   }
// }

// Control.ButtonPressed(Event e){
//  String command = e.getLabel(); //take me to source

// if (command = "takeMEToSource"){
//   Pose currentlocation = robot.getLocation();
//   Pose destination = PoseCollection.getInstance().getDestinationPose("Source");
//   Robot.getInstance().navigate(currentLocation, destination);

// }

// }
