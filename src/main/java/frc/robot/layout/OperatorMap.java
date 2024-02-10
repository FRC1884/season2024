package frc.robot.layout;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Prototypes;
import frc.robot.subsystems.Shamper;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.BoardController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeStopButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getintakeReverseButton();

  abstract JoystickButton getShootButton();

  abstract JoystickButton getShootStopButton();
  
  abstract JoystickButton getPivotButtonOne();
  
  abstract JoystickButton getPivotButtonTwo();
  
  abstract JoystickButton getFeederButton();

  abstract JoystickButton getFeederStopButton();

  abstract double getClimberAxis();



  private void registerPrototype() {
    if(ExampleConfig.Subsystems.PROTOTYPE_ENABLED) {
      Prototypes prototypes = Prototypes.getInstance();
      getShootStopButton().whileTrue(prototypes.runAny4Motors(-0.0, 0.0, 0.0, 0));
      getShootButton().whileTrue(prototypes.runAny4Motors(-15, 15, 0.0, 0));
    }
  }

    private void registerIntake() {
    if(ExampleConfig.Subsystems.INTAKE_ENABLED){
      Intake intake = Intake.getInstance();
      getIntakeStopButton().onTrue(intake.stopCommand());
      getIntakeButton().onTrue(intake.runCommand(true));
      getintakeReverseButton().onTrue(intake.runCommand(false));

    }
  }

  private void registerShamper(){
    if(ExampleConfig.Subsystems.FLYWHEEL_ENABLED){
      Shamper shamper = Shamper.getInstance();
      getShootButton().onTrue(shamper.runFlywheel(10));
      getShootStopButton().onTrue(shamper.runFlywheel(0));
      getPivotButtonOne().onTrue(shamper.runPivot(1000));
      getPivotButtonTwo().onTrue(shamper.runPivot(2000));
      getFeederButton().onTrue(shamper.runFeeder(1));
      getFeederStopButton().onTrue(shamper.runFeeder(0.0));
    }
  }

  private void registerClimber() {
    if(ExampleConfig.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();
      climber.setDefaultCommand(climber.run(this::getClimberAxis));
    }
  }

  @Override
  public void registerCommands() {
    // registerPrototype();
    registerIntake();
    registerShamper();
    registerClimber();
  }
}
