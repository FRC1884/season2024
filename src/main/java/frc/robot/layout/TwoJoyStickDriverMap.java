package frc.robot.layout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  /** <b><i>FOR {@link frc.robot.core.TalonSwerve.Swerve}</b></i>
   *
   * @return a ChassisSpeeds object that represents field-centric driving directions and powers.
   */
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    double x =
        Math.pow(controller.getLeftY(), 1)
            * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    double y =
        Math.pow(controller.getLeftX(), 1)
            * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    double rot = controller.getRightX() * MaxSwerveConstants.kMaxAngularSpeed * 0.7;

    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, new Rotation2d(0, 0));
  }

  /**
   * <b><i>FOR {@link frc.robot.core.MAXSwerve.MAXSwerve}</b></i>
   *
   * @return the robot-centric axial (forward/backward) power.
   */
  @Override
  public double getSwerveXSpeed() {
    return controller.getLeftY() * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
  }

  /**
   * <b><i>FOR {@link frc.robot.core.MAXSwerve.MAXSwerve}</b></i>
   *
   * @return the robot-centric lateral (left-right) power.
   */
  @Override
  public double getSwerveYSpeed() {
    return controller.getLeftX() * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
  }

  /**
   * <b><i>FOR {@link frc.robot.core.MAXSwerve.MAXSwerve}</b></i>
   *
   * @return the robot's up-dow-- I mean, rotational (CW-CCW) power.
   */
  @Override
  public double getSwerveRot() {
    return -controller.getRightX() * MaxSwerveConstants.kMaxAngularSpeed;
  }

  @Override
  public Trigger getTestButton() {
    return controller.b();
  }

  @Override
  public Trigger getFollowAprilTagButton() {
    return controller.a();
  }

  @Override
  public Trigger getSourceToSpeakerButton() {
    return controller.x();
  }

  @Override
  public Trigger getSourceToAmpButton() {
    return controller.y();
  }

  @Override
  public Trigger getSpeakerToSourceButton() {
    return controller.b();
  }

  @Override
  public Trigger getSpeakerToStageButton() {
    return controller.rightBumper();
  }

  @Override
  public Trigger getSpeakerOrSourceButton() {
    return controller.leftBumper();
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
