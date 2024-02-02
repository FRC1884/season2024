package frc.robot.core.util.controllers;

import java.util.HashMap;

public interface ButtonMap {
  public HashMap<Button, Integer> buttonMap();

  public HashMap<Trigger, Integer> triggerMap();

  public HashMap<Axis, Integer> axisMap();

  public HashMap<Dpad, Integer> dpadMap();

  // Joystick Buttons
  public enum Axis {
    AXIS_LEFT_X,
    AXIS_LEFT_Y,
    AXIS_RIGHT_X,
    AXIS_RIGHT_Y,
    AXIS_LEFT_TRIGGER,
    AXIS_RIGHT_TRIGGER;
  }

  // Controller Buttons
  public enum Button {
    BUTTON_A,
    BUTTON_B,
    BUTTON_X,
    BUTTON_Y,
    BUTTON_LEFT_JOYSTICK,
    BUTTON_RIGHT_JOYSTICK,
    BUTTON_LEFT_BUMPER,
    BUTTON_RIGHT_BUMPER,
    BUTTON_SHARE,
    BUTTON_OPTIONS,
    BUTTON_START,
    BUTTON_TOUCHPAD,
    BUTTON_EXTRA_1,
    BUTTON_EXTRA_2,
    BUTTON_EXTRA_3,
    BUTTON_EXTRA_4,
    BUTTON_EXTRA_5,
    BUTTON_EXTRA_6,
    BUTTON_EXTRA_7,
    BUTTON_EXTRA_8,
    BUTTON_EXTRA_9,
    BUTTON_EXTRA_10;
  }

  // Triggers
  public enum Trigger {
    BUTTON_LEFT_TRIGGER,
    BUTTON_RIGHT_TRIGGER;
  }
  

  /*
   * // Pulled from TriggerButton Class
   * public int RIGHT_TRIGGER;
   * public int LEFT_TRIGGER;
   * public int RIGHT_AXIS_UP_TRIGGER;
   * public int RIGHT_AXIS_DOWN_TRIGGER;
   * public int RIGHT_AXIS_RIGHT_TRIGGER;
   * public int RIGHT_AXIS_LEFT_TRIGGER;
   * public int LEFT_AXIS_UP_TRIGGER;
   * public int LEFT_AXIS_DOWN_TRIGGER;
   * public int LEFT_AXIS_RIGHT_TRIGGER;
   * public int LEFT_AXIS_LEFT_TRIGGER;
   */

  public enum Dpad {
    DPAD_UP,
    DPAD_UP_RIGHT,
    DPAD_RIGHT,
    DPAD_DOWN_RIGHT,
    DPAD_DOWN,
    DPAD_DOWN_LEFT,
    DPAD_LEFT,
    DPAD_UP_LEFT;
  }
}
