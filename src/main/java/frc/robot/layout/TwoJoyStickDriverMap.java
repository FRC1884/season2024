package frc.robot.layout;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.RobotMap.DriveMap;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public double getSwerveXSpeed() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return -controller.getAxis(Axis.AXIS_LEFT_Y) * (MaxSwerveConstants.kMaxSpeedMetersPerSecond)
          * ((DriveMap.IS_SLOWMODE_ENABLED) ? DriveMap.SLOW_MODE_TRANSLATE_MULTIPLIER : 1);
    } else {
      return controller.getAxis(Axis.AXIS_LEFT_Y) * (MaxSwerveConstants.kMaxSpeedMetersPerSecond)
          * ((DriveMap.IS_SLOWMODE_ENABLED) ? DriveMap.SLOW_MODE_TRANSLATE_MULTIPLIER : 1);
    }
  }

  @Override
  public double getSwerveYSpeed() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return -controller.getAxis(Axis.AXIS_LEFT_X) * (MaxSwerveConstants.kMaxSpeedMetersPerSecond)
          * ((DriveMap.IS_SLOWMODE_ENABLED) ? DriveMap.SLOW_MODE_TRANSLATE_MULTIPLIER : 1);
    } else {
      return controller.getAxis(Axis.AXIS_LEFT_X) * (MaxSwerveConstants.kMaxSpeedMetersPerSecond)
          * ((DriveMap.IS_SLOWMODE_ENABLED) ? DriveMap.SLOW_MODE_TRANSLATE_MULTIPLIER : 1);
    }
  }

  @Override
  public double getSwerveRot() {
    return -controller.getAxis(Axis.AXIS_RIGHT_X) * MaxSwerveConstants.kMaxAngularSpeed
        * ((DriveMap.IS_SLOWMODE_ENABLED) ? DriveMap.SLOW_MODE_ROTATION_MUTLIPLIER : 0.3);
  }

  @Override
  public Trigger getSlowModeToggleButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public Trigger getArcingButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  public Trigger getTestButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  public Trigger getFollowAprilTagButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public Trigger getFollowNoteButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  public Trigger getZeroGyroButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public Trigger getNavigateAndAllignAmpButton() {
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_TRIGGER) > 0.5);
  }

  @Override
  public Trigger getNavigateAndAllignStageButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public Trigger getFerryArcButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
