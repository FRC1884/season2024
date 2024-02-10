package frc.robot.core.util.controllers;

public abstract class CommandMap {

  protected GameController controller;

  public CommandMap(GameController c) {
    controller = c;
  }

  public abstract void registerCommands();
}
