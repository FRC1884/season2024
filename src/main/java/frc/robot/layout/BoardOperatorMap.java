package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class BoardOperatorMap extends OperatorMap {
  public BoardOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  JoystickButton getOuttakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getShootSpeakerButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getShootAmpButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  JoystickButton getAmpAlignButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  JoystickButton getChangeClimberLockStatusButton(){
    return controller.getButton(Button.BUTTON_EXTRA_3);
  }

  @Override
  double getManualPivotAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  double getManualClimberAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_X);
  }

  @Override
  JoystickButton getManualShootButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getClimbSequenceButton() {
    return controller.getButton(Button.BUTTON_TOUCHPAD);
  }

  @Override
  JoystickButton getLEDPatternOffButton() {
    return controller.getButton(Button.BUTTON_EXTRA_2);
  }

  @Override
  double getLEDAxis1() {
    return controller.getAxis(Axis.AXIS_LEFT_X);
  }

  @Override
  double getLEDAxis2() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  JoystickButton getSpeakerShotAlignButton() {
    return controller.getButton(Button.BUTTON_SHARE);
  }

  @Override
  JoystickButton getFerryShotAlignButton() {
    return controller.getButton(Button.BUTTON_EXTRA_10); //TODO: BIND TO ANOTHER BUTTON
  }

  @Override
  JoystickButton getTrapButton() {
    return controller.getButton(Button.BUTTON_START);
  }

  @Override
  JoystickButton getStageAlignButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getAmplifyButton() {
    return controller.getButton(Button.BUTTON_EXTRA_1);
  }

  @Override
  JoystickButton getCoopButton() {
    return controller.getButton(Button.BUTTON_EXTRA_2);
  }

  @Override
  Trigger getPivotRaiseButton() {
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_X) < -0.9);
  }

  @Override
  Trigger getPivotLowerButton() {
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_X) > 0.9);
  }

  @Override
  Trigger getClimberRaiseButton() {
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_Y) > 0.9);
  }

  @Override
  Trigger getClimberLowerButton() {
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_Y) < -0.9);
  }

  @Override
  JoystickButton getEjectButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  JoystickButton getSubwooferShotButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  JoystickButton getPodiumShotButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);

  }
}