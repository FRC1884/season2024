package frc.robot.layout;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.subsystems.*;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.BoardController;
import frc.robot.util.BlinkinUtils;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getintakeReverseButton();

  abstract JoystickButton getShootButton();

  abstract JoystickButton getShootAmpButton();
  
  abstract JoystickButton getPivotButtonOne();
  
  abstract JoystickButton getPivotButtonTwo();
  
  abstract JoystickButton getFeederButton();

  abstract double getClimberAxis();

  abstract double getManualPivotAxis();

  abstract JoystickButton getLEDPatternOneButton();

  abstract JoystickButton getLEDPatternTwoButton();

  abstract JoystickButton getLEDPatternOffButton();


  private void registerPrototype() {
    if(ExampleConfig.Subsystems.PROTOTYPE_ENABLED) {
      Prototypes prototypes = Prototypes.getInstance();
      
      getShootButton().whileTrue(prototypes.runAny4Motors(-0.30, 0.30, 0.0, 0));
      getFeederButton().whileTrue(prototypes.runAny4Motors(0.0,0.0,-0.1,0.0));

    }
  }

    private void registerIntake() {
    if(ExampleConfig.Subsystems.INTAKE_ENABLED){
      Intake intake = Intake.getInstance();
      Shamper shamper = Shamper.getInstance();
      getIntakeButton().onTrue(intake.runCommand(false));
      getIntakeButton().onFalse(intake.stopCommand());
      getintakeReverseButton().onTrue(intake.runCommand(true));
      getintakeReverseButton().onFalse(intake.stopCommand());
    }
  }

  private void registerShamper(){
    if(ExampleConfig.Subsystems.SHAMPER_ENABLED){
      Shamper shamper = Shamper.getInstance();
      Pivot pivot = Pivot.getInstance();

      getShootButton().whileTrue(shamper.runFlywheel(10));
      getShootButton().whileFalse(shamper.runFlywheel(0.0));
      getShootAmpButton().whileTrue(shamper.runFlywheel(5));
      getShootAmpButton().whileFalse(shamper.runFlywheel(0.0));
      getFeederButton().whileTrue(shamper.runFeederPower(0.9, false));
      getFeederButton().whileFalse(shamper.runFeederPower(0, false));
      // getPivotButtonOne().onTrue(pivot.runPivotPower(() -> 0.2));
      // getPivotButtonOne().onFalse(shamper.runPivotPower(() -> 0.0));
      getPivotButtonTwo().onTrue(pivot.moveTo(2000));
      getPivotButtonTwo().onFalse(pivot.moveTo(0));
    }
  }

  private void registerClimber() {
    if(ExampleConfig.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();
      climber.setDefaultCommand(climber.run(this::getClimberAxis));
    }
  }

  private void registerLEDs() {
    if(ExampleConfig.Subsystems.LEDS_ENABLED) {
      PWMLEDLights lights = PWMLEDLights.getInstance();
      getLEDPatternOffButton().onTrue(
              lights.setColorForSecondsCommand(3, BlinkinUtils.ColorPatterns.WHITE)
      );
      getLEDPatternOneButton().onTrue(
              lights.setColorCommand(BlinkinUtils.ColorPatterns.SINELON_RAINBOW_PALETTE)
      );
      getLEDPatternTwoButton().onTrue(
              lights.setColorCommand(BlinkinUtils.ColorPatterns.CP1_2_END_TO_END_BLEND)
      );
    }
  }

  @Override
  public void registerCommands() {
    //registerPrototype();
    registerIntake();
    registerShamper();
    // registerClimber();
    // registerLEDs();  
  }
}
