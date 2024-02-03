package frc.robot.core.util.controllers;

import java.util.HashMap;

import javax.xml.namespace.QName;

import org.opencv.objdetect.Board;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;

public class BoardController extends CommandGenericHID { 
  public BoardController(int port) {
    super(port);
    
    //TODO Auto-generated constructor stub
  }


    public HashMap<Button, Integer> buttonMap() {
      var map = new HashMap<Button, Integer>();
  
      map.put(Button.BUTTON_A, 6);
      map.put(Button.BUTTON_B, 7);
      map.put(Button.BUTTON_X, 8);
      map.put(Button.BUTTON_Y, 9);
      map.put(Button.BUTTON_LEFT_BUMPER, 10);
      map.put(Button.BUTTON_RIGHT_BUMPER, 11);
      map.put(Button.BUTTON_SHARE, 12);
      map.put(Button.BUTTON_OPTIONS, 13);
      map.put(Button.BUTTON_START, 14);
      map.put(Button.BUTTON_TOUCHPAD, 15);
      map.put(Button.BUTTON_EXTRA_1, 16);
      map.put(Button.BUTTON_EXTRA_2, 5);
  
      return map;
  }
  public HashMap<Axis, Integer> AxisMap() {
      var map = new HashMap<Axis, Integer>();
      map.put(Axis.AXIS_LEFT_X, 0);
      map.put(Axis.AXIS_LEFT_Y, 1);
  
      return map;
  }
  

  

}
