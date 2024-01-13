package frc.robot.core.TalonSwerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class Swerve extends SubsystemBase {
  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;
  private Pigeon2 gyro;

  public Swerve(
      int pigeon_id,
      SwerveModuleConstants fl,
      SwerveModuleConstants fr,
      SwerveModuleConstants bl,
      SwerveModuleConstants br) {
    gyro = new Pigeon2(pigeon_id);
    gyro.getConfigurator().DefaultTimeoutSeconds = 50;
    zeroGyro();

    modules = new SwerveModule[] {
        new SwerveModule(0, fl),
        new SwerveModule(1, fr),
        new SwerveModule(2, bl),
        new SwerveModule(3, br)
    };

    odometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS, getYaw(), getModulePositions());

    var swerveTab = Shuffleboard.getTab("Swerve");

    swerveTab.addDouble("module 0 position", () -> getModulePositions()[0].distanceMeters);
    swerveTab.addDouble("module 1 position", () -> getModulePositions()[1].distanceMeters);
    swerveTab.addDouble("module 2 position", () -> getModulePositions()[2].distanceMeters);
    swerveTab.addDouble("module 3 position", () -> getModulePositions()[3].distanceMeters);
    for (SwerveModule mod : modules) {
      swerveTab.addDouble(
          "Mod " + mod.moduleNumber + " Cancoder", () -> mod.getCanCoder().getDegrees());
      swerveTab.addDouble(
          "Mod " + mod.moduleNumber + " Integrated", () -> mod.getPosition().angle.getDegrees());
      swerveTab.addDouble(
          "Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
    }
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : modules) {
      mod.resetToAbsolute();
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds speeds = SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    return speeds;
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_VELOCITY);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }
  }

  public Command driveCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
    return new RepeatCommand(new RunCommand(() -> this.drive(chassisSpeeds.get()), this));
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_VELOCITY);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : modules) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : modules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
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
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        getShouldFlip(),
        this // Reference to this subsystem to set requirements
    );
  }
  public Command followAprilTagCommand() {
    return new RepeatCommand(
        new RunCommand(() -> this.followPathCommand(
          new PathPlannerPath(PathPlannerPath.bezierFromPoses(getPose(),getPose()), null, null) // null vaules because these are to be obtained from vision when that is finished
        ), this));
  }

  public BooleanSupplier getShouldFlip() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValue())
        : Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      resetModulesToAbsolute();
    }

    odometry.update(getYaw(), getModulePositions());
  }
}
