package frc.robot.core.util.controllers;

import java.util.HashMap;

public class Logitech implements ButtonMap {
  public enum Version {
    REGULAR,
    REVERSED_TRIGGER_AND_STICK
  }

  Version version; // Version 1 or Version 2

  public Logitech(Version version) {
    this.version = version;
  }

  @Override
  public HashMap<Button, Integer> buttonMap() {
    var map = new HashMap<Button, Integer>();

    map.put(Button.BUTTON_X, 1);
    map.put(Button.BUTTON_A, 2);
    map.put(Button.BUTTON_B, 3);
    map.put(Button.BUTTON_Y, 4);
    map.put(Button.BUTTON_LEFT_JOYSTICK, 11);
    map.put(Button.BUTTON_RIGHT_JOYSTICK, 12);
    map.put(Button.BUTTON_LEFT_BUMPER, 5);
    map.put(Button.BUTTON_RIGHT_BUMPER, 6);
    map.put(Button.BUTTON_SHARE, 9); // Back Button
    map.put(Button.BUTTON_OPTIONS, 10);
    map.put(Button.BUTTON_START, 10);
    // map.put(Button.BUTTON_TOUCHPAD, 11);

    return map;
  }

  @Override
  public HashMap<Trigger, Integer> triggerMap() {
    var map = new HashMap<Trigger, Integer>();

    map.put(Trigger.BUTTON_LEFT_TRIGGER, 7);
    map.put(Trigger.BUTTON_RIGHT_TRIGGER, 8);

    return map;
  }

  @Override
  public HashMap<Axis, Integer> axisMap() {
    switch (version) {
      case REGULAR:
        return axisMapRegular();
      case REVERSED_TRIGGER_AND_STICK:
        return axisMapTriggerAndStick();
      default:
        return axisMapRegular();
    }
  }

  private HashMap<Axis, Integer> axisMapRegular() {
    var map = new HashMap<Axis, Integer>();
    map.put(Axis.AXIS_LEFT_X, 0);
    map.put(Axis.AXIS_LEFT_Y, 1);
    map.put(Axis.AXIS_RIGHT_X, 2);
    map.put(Axis.AXIS_RIGHT_Y, 3);
    map.put(Axis.AXIS_LEFT_TRIGGER, 4);
    map.put(Axis.AXIS_RIGHT_TRIGGER, 5);
    return map;
  }

  private HashMap<Axis, Integer> axisMapTriggerAndStick() {
    var map = new HashMap<Axis, Integer>();
    map.put(Axis.AXIS_LEFT_X, 0);
    map.put(Axis.AXIS_LEFT_Y, 1);
    map.put(Axis.AXIS_LEFT_TRIGGER, 2);
    map.put(Axis.AXIS_RIGHT_TRIGGER, 3);
    map.put(Axis.AXIS_RIGHT_X, 4);
    map.put(Axis.AXIS_RIGHT_Y, 5);
    return map;
  }

  @Override
  public HashMap<Dpad, Integer> dpadMap() {
    var map = new HashMap<Dpad, Integer>();

    map.put(Dpad.DPAD_UP, 0);
    map.put(Dpad.DPAD_UP_RIGHT, 45);
    map.put(Dpad.DPAD_RIGHT, 90);
    map.put(Dpad.DPAD_DOWN_RIGHT, 135);
    map.put(Dpad.DPAD_DOWN, 180);
    map.put(Dpad.DPAD_DOWN_LEFT, 225);
    map.put(Dpad.DPAD_LEFT, 270);
    map.put(Dpad.DPAD_UP_LEFT, 315);

    return map;
  }
}
