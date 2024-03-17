package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  public Trigger getClimberRaiseButton(){
    return null;
  }

  @Override
  public Trigger getClimberLowerButton(){
    return null;
  }

  @Override
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getShootSpeakerButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getShootAmpButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getOuttakeButton() {
    return controller.getButton(Button.BUTTON_SHARE);
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
  double getManualClimberAxis(){
    return 0;
  }



  
  @Override
  double getLEDAxis1() {
    return controller.getAxis(Axis.AXIS_LEFT_X);
  }

  @Override
  JoystickButton getEjectButton(){
    return null;
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

  @Override
  JoystickButton getSubwooferShotButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSubwooferShotButton'");
  }

  @Override
  JoystickButton getPodiumShotButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAmpShotButton'");
  }
}
