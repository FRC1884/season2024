package frc.robot.core.MAXSwerve;

import static frc.robot.core.TalonSwerve.SwerveConstants.KINEMATICS;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.core.MAXSwerve.MaxSwerveConstants.*;
import frc.robot.core.TalonSwerve.SwerveConstants;
import frc.robot.subsystems.Vision.PoseEstimator;
import frc.robot.subsystems.Vision.Vision;

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
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain Debugging");
  GenericEntry targetAngleEntry = tab.add("Target Angle", 0).getEntry();
  GenericEntry currentAngleEntry = tab.add("Current Angle", 0).getEntry();
  private double targetAngleTelemetry = 0;

  SwerveDriveOdometry odometry;

  //For Testing All functions
  private double startTime, currentTime;

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

    odometry = new SwerveDriveOdometry(
        MaxSwerveConstants.kDriveKinematics,
        getYaw(),
        new SwerveModulePosition[] {
            fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
        });
    var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
    //   this.resetOdometry(new Pose2d(15, 5.18, Rotation2d.fromDegrees(0)));
    // else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue)
    //   this.resetOdometry(new Pose2d(15, 5.18, Rotation2d.fromDegrees(0)));
    // else
    //   this.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    //System.out.println(odometry.getPoseMeters());
    //System.out.println(MathUtil.inputModulus(this.getYaw().getDegrees(), -180, 180));
    odometry.update(
        getYaw(),
        new SwerveModulePosition[] {
            fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
        });
    //resetOdometry(PoseEstimator.getInstance().getPosition()); //NEEDS MORE TESTING
    targetAngleEntry.setDouble(targetAngleTelemetry);
    currentAngleEntry.setDouble(getHeading() % 360);
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
        getYaw(),
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
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = MAXSwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = MAXSwerveUtils.StepTowardsCircular(
            currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = MAXSwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir = MAXSwerveUtils.StepTowardsCircular(
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
    var swerveModuleStates = MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);

    fl.setDesiredState(swerveModuleStates[0]);
    fr.setDesiredState(swerveModuleStates[1]);
    bl.setDesiredState(swerveModuleStates[2]);
    br.setDesiredState(swerveModuleStates[3]);
  }

  public Command driveSetAngleCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Pose2d> targetPose) {
    PIDController pid = new PIDController(0.01, 0, 0);
    pid.setTolerance(0.1);
    return new ProxyCommand(() ->
      new RepeatCommand(
        new FunctionalCommand(
          () -> {
            // Init
          },
          () -> {
            double targetX = targetPose.get().getX();
            double targetY = targetPose.get().getY();
            double targetAngle = Math.toDegrees(Math.atan2((targetY-this.getPose().getY()),(targetX-this.getPose().getX())));
            double robotAngle = this.getYaw().getDegrees();
            
            //
            if (targetX-this.getPose().getX() <= 0 && targetY-this.getPose().getY() >= 0) {
              targetAngle = -Math.toDegrees(Math.abs(Math.atan(targetY-this.getPose().getY())/(targetX-this.getPose().getX())));
              System.out.println(1);
            }
            if (targetX-this.getPose().getX() > 0 && targetY-this.getPose().getY() > 0) {
              targetAngle = -90-(90-Math.toDegrees(Math.abs(Math.atan(targetY-this.getPose().getY())/(targetX-this.getPose().getX()))));
              System.out.println(2);
            }

            if (targetX-this.getPose().getX() > 0 && targetY-this.getPose().getY() == 0) {
              targetAngle = robotAngle;
            }

            if (targetX-this.getPose().getX() >= 0 && targetY-this.getPose().getY() <= 0) {
              targetAngle = 180 - Math.toDegrees(Math.abs(Math.atan(targetY-this.getPose().getY()/targetX-this.getPose().getX())));
              System.out.println(3);
            }
            if (targetX-this.getPose().getX() < 0 && targetY-this.getPose().getY() < 0) {
              targetAngle = 90 - (90-Math.toDegrees(Math.abs(Math.atan(targetY-this.getPose().getY())/(targetX-this.getPose().getX()))));
              System.out.println(4);
            }
            targetAngleTelemetry = targetAngle;
            //System.out.println(targetAngle);
            if (Math.abs(pid.calculate(MathUtil.inputModulus(robotAngle,-180,180),
                targetAngle)) > RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY) {
                this.drive(xSpeed.get(),ySpeed.get(), RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY, 
                  true, true);
            } else {
              double newSpeed = pid.calculate(MathUtil.inputModulus(robotAngle,-180,180), 
                  targetAngle);
                  this.drive(xSpeed.get(),ySpeed.get(),
                  newSpeed, true, true);
                  // System.out.println(newSpeed + "," + MathUtil.inputModulus(this.getYaw().getDegrees(),-180,180) + "," 
                  // + targetAngle );
            }
            
              },
              interrupted -> {
                pid.close();
                this.drive(0.0,0.0,0.0,true,true);
              },
              () -> {
                return pid.atSetpoint();
              },
              this))
    );
  }

  public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
    fl.setDesiredState(swerveModuleStates[0]);
    fr.setDesiredState(swerveModuleStates[1]);
    bl.setDesiredState(swerveModuleStates[2]);
    br.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {

    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(fl.getState(), fr.getState(), bl.getState(), br.getState());

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

  public Command followPathCommand(String pathName, boolean isFirstPath) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return followPathCommand(path, isFirstPath);
  }

  public Command followPathCommand(PathPlannerPath pathName, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                PathPoint startingPoint = pathName.getPoint(0);
                Pose2d startingPose = new Pose2d(
                    startingPoint.position, Rotation2d.fromDegrees(180));
                this.resetOdometry(startingPose);
              }
            }),
        new FollowPathHolonomic(
            pathName,
            this::getPose, // Robot pose supplier
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveWithChassisSpeeds,
            /*speeds -> driveWithChassisSpeeds(
              new ChassisSpeeds(
                speeds.vxMetersPerSecond * Math.cos(gyro.getYaw()) - speeds.vyMetersPerSecond * Math.sin(gyro.getYaw()),
                speeds.vxMetersPerSecond * Math.sin(gyro.getYaw()) + speeds.vyMetersPerSecond * Math.cos(gyro.getYaw()),
                gyro.getYaw()
              )
            ),*/
            // Method that will drive the robot given ROBOT RELATIVE
            // ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                // in
                // your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                MaxSwerveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the
            // options
            // here
            ),
            getShouldFlip(),
            this // Reference to this subsystem to set requirements
        ));
  }

  public Command followAprilTagCommand() {
    return new RepeatCommand(
        new RunCommand(
            () -> this.followPathCommand(
                new PathPlannerPath(
                    PathPlannerPath.bezierFromPoses(
                        Vision.getInstance().getRobotPose2d_TargetSpace(),
                        new Pose2d(1.0, 0.0, new Rotation2d())), // Need to make this better
                    null,
                    null),
                false // null vaules because these are to be obtained from vision when that
            // is finished
            ),
            this));
  }

  /**
   * Command to drive to a note detected using Vision - NEEDS TO BE REWORKED TO USE NAVIGATE
   * Preivous ERROR: Robot travels to the robot relative note pose in field relative coordinates
   * FIX: PARAMETER MUST BE A SUPPLIER SO IT CONTINUOUSLY UPDATES
   * @param targetPose the Supplier<Pose2d> that the robot should drive to
   * @return command to generate a path On-the-fly to a note
   */
  public Command onTheFlyPathCommand(Supplier<Pose2d> targetPose) {
    return new ProxyCommand(() -> followPathCommand(
        new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(new Pose2d(this.getPose().getTranslation(),
                                                Rotation2d.fromDegrees(0)),
                                            new Pose2d(targetPose.get().getTranslation(),
                                            Rotation2d.fromDegrees(0))),
            new PathConstraints(
                RobotMap.SwervePathFollowConstants.MAX_VELOCITY,
                RobotMap.SwervePathFollowConstants.MAX_ACCELERATION,
                RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY,
                RobotMap.SwervePathFollowConstants.MAX_ANG_ACCELERATION),
            new GoalEndState(0, targetPose.get().getRotation()),
            false),
        false));
  }

  /**
   * Command to pathfind to a path - NEEDS TO BE TESTED, also, does it need to be a proxy?
   * @param pathName name of the path that the robot will pathfind to  
   * @return command to generate a path to the starting pose of a premade path and then to follow that path
   */

  public Command pathFindThenFollowPathCommand(String pathName){
    return new ProxyCommand(new PathfindThenFollowPathHolonomic(
        PathPlannerPath.fromPathFile(pathName),
        new PathConstraints(
                RobotMap.SwervePathFollowConstants.MAX_VELOCITY,
                RobotMap.SwervePathFollowConstants.MAX_ACCELERATION,
                RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY,
                RobotMap.SwervePathFollowConstants.MAX_ANG_ACCELERATION),
        this::getPose,
        this::getChassisSpeeds,
        this::driveWithChassisSpeeds,
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                // in
                // your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the
            // options
            // here
            ),
        1.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
            
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            getShouldFlip(),
            
        this // Reference to drive subsystem to set requirements
      )
    );
  }

  public Command navigate(Supplier<Pose2d> targetPose, Supplier<String> pathName) {
    return followPathCommand(
        new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(new Pose2d(this.getPose().getTranslation(),
                                                Rotation2d.fromDegrees(0)),
                                            new Pose2d(targetPose.get().getTranslation(),
                                            Rotation2d.fromDegrees(0))),
            new PathConstraints(
                RobotMap.SwervePathFollowConstants.MAX_VELOCITY,
                RobotMap.SwervePathFollowConstants.MAX_ACCELERATION,
                RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY,
                RobotMap.SwervePathFollowConstants.MAX_ANG_ACCELERATION),
            new GoalEndState(0, new Rotation2d()),
            false),
        false);
  }

  // public Command goSpeakerOrSource(boolean hasNote) {
  //   if (hasNote) {
  //     return navigate(getPose(),"WithNote");
  //   } else {
  //     return navigate(getPose(), "NoNote");
  //   }
  // }

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

  public SequentialCommandGroup TestAllCommand() {
    return new SequentialCommandGroup(
      new FunctionalCommand(
            () -> {
              startTime = Timer.getFPGATimestamp();},
            () -> {
              drive(0.1, 0.1, 0, true, true);
              currentTime = Timer.getFPGATimestamp();
      },
            interrupted -> {
              drive(0, 0, 0, true, true);
      },
      () -> {return (currentTime - startTime >= 0.5);},
    this
        ),
      new WaitCommand(1),
      new FunctionalCommand(
            () -> {
              startTime = Timer.getFPGATimestamp();},
            () -> {
              drive(0, 0, 0.1, true, true);
              currentTime = Timer.getFPGATimestamp();
      },
            interrupted -> {
              drive(0, 0, 0, true, true);
      },
      () -> {return (currentTime - startTime >= 0.5);},
    this
        ),
      new WaitCommand(1),

      navigate(() -> getPose().plus(new Transform2d(new Translation2d(1, 0), new Rotation2d())),
            () -> "Testing Translation X"),
      new WaitCommand(1),
      navigate(() -> getPose().plus(new Transform2d(new Translation2d(0, 1), new Rotation2d())), () -> "Testing Translation Y"),
      new WaitCommand(1),
      navigate(() -> getPose().plus(new Transform2d(new Translation2d(0, 1), new Rotation2d(90))), () -> "Testing Rotation"),
      new WaitCommand(1),
      followPathCommand("ShortTestPath", true)
    );
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
