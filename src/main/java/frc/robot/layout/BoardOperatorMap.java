package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class BoardOperatorMap extends OperatorMap {
  public BoardOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  Trigger getIntakeButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  Trigger getOuttakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  Trigger getShootSpeakerButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getShootAmpButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  Trigger getAmpAlignButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  Trigger getBackPodiumShotButton(){
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
  Trigger getClimbSequenceButton() {
    return controller.getButton(Button.BUTTON_TOUCHPAD);
  }

  @Override
  Trigger getLEDPatternOffButton() {
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
  Trigger getSpeakerShotAlignButton() {
    return controller.getButton(Button.BUTTON_SHARE);
  }

  @Override
  Trigger getFerryShotAlignButton() {
    return controller.getButton(Button.BUTTON_EXTRA_10); //TODO: BIND TO ANOTHER BUTTON
  }

  @Override
  Trigger getTrapButton() {
    return controller.getButton(Button.BUTTON_START);
  }

  @Override
  Trigger getStageAlignButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  Trigger getAmplifyButton() {
    return controller.getButton(Button.BUTTON_EXTRA_1);
  }

  @Override
  Trigger getCoopButton() {
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
  Trigger getEjectButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  Trigger getSubwooferShotButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  Trigger getPodiumShotButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);

  }

  @Override
  Trigger getSourceIntakeButton() {
    // TODO Auto-generated method stub
    return controller.getButton(Button.BUTTON_START);
  }
}