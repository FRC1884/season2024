package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickOperatorMap extends OperatorMap {
  public TwoJoyStickOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }

  @Override
  public Trigger getClimberRaiseButton() {
    return null;
  }

  @Override
  public Trigger getClimberLowerButton() {
    return null;
  }

  @Override
  public Trigger getIntakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  Trigger getShootSpeakerButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getShootAmpButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  Trigger getOuttakeButton() {
    return controller.getButton(Button.BUTTON_SHARE);
  }

  @Override
  Trigger getAmpAlignButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  double getManualPivotAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  Trigger getLEDPatternOffButton() {
    return null;
  }

  @Override
  Trigger getChangeClimberLockStatusButton(){
    return null;
  }

  @Override
  Trigger getFerryShotAlignButton() {
    return null;
  }

  @Override
  double getManualClimberAxis() {
    return 0;
  }

  @Override
  double getLEDAxis1() {
    return controller.getAxis(Axis.AXIS_LEFT_X);
  }

  @Override
  Trigger getEjectButton() {
    return null;
  }

  @Override
  double getLEDAxis2() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  Trigger getSpeakerShotAlignButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  Trigger getPivotRaiseButton() {
    return null;
  }

  @Override
  Trigger getPivotLowerButton() {
    return null;
  }

  @Override
  Trigger getTrapButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  Trigger getStageAlignButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  Trigger getAmplifyButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  Trigger getCoopButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  Trigger getClimbSequenceButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  Trigger getSubwooferShotButton() {
    // TODO Auto-generated method stub
    throw new Error("Unimplemented method 'getSubwooferShotButton'");
  }

  @Override
  Trigger getPodiumShotButton() {
    // TODO Auto-generated method stub
    throw new Error("Unimplemented method 'getAmpShotButton'");
  }

  @Override
  Trigger getSourceIntakeButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSourceIntakeButton'");
  }
}
