package frc.robot.core.util.controllers;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;

public class BoardController implements ButtonMap {  //TODO Auto-generated constructor stub
  


    public HashMap<Button, Integer> buttonMap() {
      var map = new HashMap<Button, Integer>();
      map.put(Button.BUTTON_A, 1);
      map.put(Button.BUTTON_B, 2);
      map.put(Button.BUTTON_X, 3);
      map.put(Button.BUTTON_Y, 4);
      map.put(Button.BUTTON_LEFT_BUMPER, 5);
      map.put(Button.BUTTON_RIGHT_BUMPER, 6);
      map.put(Button.BUTTON_SHARE, 7);
      map.put(Button.BUTTON_OPTIONS, 8);
      map.put(Button.BUTTON_START, 9);
      map.put(Button.BUTTON_TOUCHPAD, 10);
      map.put(Button.BUTTON_EXTRA_1, 11);
      map.put(Button.BUTTON_EXTRA_2, 12);
  
      return map;
  }
  public HashMap<Axis, Integer> axisMap() {
      var map = new HashMap<Axis, Integer>();
      map.put(Axis.AXIS_LEFT_X, 0);
      map.put(Axis.AXIS_LEFT_Y, 1);
  
      return map;
  }
  @Override
  public HashMap<Trigger, Integer> triggerMap() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'triggerMap'");
  }
  @Override
  public HashMap<Dpad, Integer> dpadMap() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'dpadMap'");
  }


  

}
