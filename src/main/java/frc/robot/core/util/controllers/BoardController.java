package frc.robot.core.util.controllers;

import java.util.HashMap;

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
      map.put(Button.BUTTON_EXTRA_3, 13);
      map.put(Button.BUTTON_EXTRA_4, 14);
      map.put(Button.BUTTON_EXTRA_5, 15);
      map.put(Button.BUTTON_EXTRA_6, 16);
      map.put(Button.BUTTON_EXTRA_7, 17);
      map.put(Button.BUTTON_EXTRA_8, 18);
      map.put(Button.BUTTON_EXTRA_9, 19);
      map.put(Button.BUTTON_EXTRA_10, 20);
      
  
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
