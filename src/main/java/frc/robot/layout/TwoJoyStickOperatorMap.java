package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  public JoystickButton getIntakeStopButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getShootButton() {
    return controller.getButton(Button.BUTTON_START);
  }

  JoystickButton getShootStopButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getShootAmpButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  JoystickButton getIntakeReverseButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getAmpAlignButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  double getManualPivotAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  JoystickButton getLEDPatternOneButton() {
    return null;
  }

  @Override
  JoystickButton getLEDPatternTwoButton() {
    return null;
  }

  @Override
  JoystickButton getLEDPatternOffButton() {
    return null;
  }

  @Override
  JoystickButton getLEDPatternThreeButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getLEDPatternFourButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getLEDPatternFiveButton() {
    // TODO Auto-generated method stub
    return null;
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
  JoystickButton getManualShootButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getArcButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getPivotRaiseButton()
  {
    return null;
  }

  @Override
  JoystickButton getPivotLowerButton(){
    return null;
  }

  @Override
  JoystickButton getTrapButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getStageAlignButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getAmplifyButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getCoopButton() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  JoystickButton getClimbSequenceButton() {
    // TODO Auto-generated method stub
    return null;
  }
}
