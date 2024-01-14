package frc.robot.core.util.controllers;

import java.util.HashMap;

public class BoardController implements ButtonMap {
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
        map.put(Button.BUTTON_SHARE, 9);
        map.put(Button.BUTTON_OPTIONS, 10);
        map.put(Button.BUTTON_START, 13);
        map.put(Button.BUTTON_TOUCHPAD, 14);

        return map;
    }

    @Override
    public HashMap<Trigger, Integer> triggerMap() {
        return null;
    }

    @Override
    public HashMap<Axis, Integer> axisMap() {
        return null;
    }

    @Override
    public HashMap<Dpad, Integer> dpadMap() {
        return null;
    }
}
