package frc.robot.core.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
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
import java.util.function.Supplier;

public abstract class Swerve extends SubsystemBase {
  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;
  private WPI_Pigeon2 gyro;

  public Swerve(
      int pigeon_id,
      SwerveModuleConstants fl,
      SwerveModuleConstants fr,
      SwerveModuleConstants bl,
      SwerveModuleConstants br) {
    gyro = new WPI_Pigeon2(pigeon_id);
    gyro.configFactoryDefault();
    zeroGyro();

    modules =
        new SwerveModule[] {
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

  public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_VELOCITY);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public Command driveCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
    return new RepeatCommand(new RunCommand(() -> this.drive(chassisSpeeds.get(), true), this));
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

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      resetModulesToAbsolute();
    }

    odometry.update(getYaw(), getModulePositions());
  }
}
