package frc.robot.layout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.core.TalonSwerve.SwerveConstants;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    double x = Math.pow(controller.getAxis(Axis.AXIS_LEFT_Y), 1) * SwerveConstants.MAX_VELOCITY;
    double y = Math.pow(controller.getAxis(Axis.AXIS_LEFT_X), 1) * SwerveConstants.MAX_VELOCITY;
    double rot = controller.getAxis(Axis.AXIS_RIGHT_X) * SwerveConstants.MAX_ANGULAR_VELOCITY * 0.7;

    return ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, -rot, new Rotation2d(0, 0));
  }

  @Override
  public double getSwerveXSpeed() {
    return controller.getAxis(Axis.AXIS_LEFT_Y) * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
  }

  @Override
  public double getSwerveYSpeed() {
    return controller.getAxis(Axis.AXIS_LEFT_X) * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
  }

  @Override
  public double getSwerveRot() {
    return -controller.getAxis(Axis.AXIS_RIGHT_X) * MaxSwerveConstants.kMaxAngularSpeed;
  }

  @Override
  public JoystickButton getTestButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  public JoystickButton getFollowAprilTagButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public JoystickButton getSourceToSpeakerButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public JoystickButton getSourceToAmpButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  public JoystickButton getSpeakerToSourceButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  public JoystickButton getSpeakerToStageButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  public JoystickButton getSpeakerOrSourceButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
