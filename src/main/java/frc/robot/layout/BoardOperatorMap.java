package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.BoardController;
import frc.robot.core.util.controllers.GameController;

public class BoardOperatorMap extends OperatorMap {
    public BoardOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }

  public JoystickButton getBoardButtonOne(){
    return controller.getButton(Button.BUTTON_EXTRA_2);
  }
  public JoystickButton getBoardButtonTwo(){
    return controller.getButton(Button.BUTTON_A);
  }
  public JoystickButton getBoardButtonThree(){
    return controller.getButton(Button.BUTTON_B);
  }
  public JoystickButton getBoardButtonFour(){
    return controller.getButton(Button.BUTTON_X);
  }
  public JoystickButton getBoardButtonFive(){
    return controller.getButton(Button.BUTTON_Y);
  }
  public JoystickButton getBoardButtonSix(){
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }
  public JoystickButton getBoardButtonSeven(){
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }
  public JoystickButton getBoardButtonEight(){
    return controller.getButton(Button.BUTTON_SHARE);
  }
  public JoystickButton getBoardButtonNine(){
    return controller.getButton(Button.BUTTON_OPTIONS);
  }
  public JoystickButton getBoardButtonTen(){
    return controller.getButton(Button.BUTTON_START);
  }
  public JoystickButton getBoardButtonEleven(){
    return controller.getButton(Button.BUTTON_TOUCHPAD);
  }
  public JoystickButton getBoardButtonTwelve(){
    return controller.getButton(Button.BUTTON_EXTRA_1); 
  }
  public Trigger getBoardAxisX(){
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_X) > 0.9);
  }
  public Trigger getBoardAxisY(){
    return new Trigger(() -> controller.getAxis(Axis.AXIS_LEFT_Y) > 0.9);
  }

  @Override
  JoystickButton getResetButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getResetButton'");
  }

  @Override
  JoystickButton getIntakeButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getIntakeButton'");
  }

  @Override
  JoystickButton getShootButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getShootButton'");
  }

  
}
