package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.PWMLEDLights;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Prototypes;
import frc.robot.subsystems.Shamper;
import frc.robot.util.BlinkinUtils;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeStopButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getIntakeReverseButton();

  abstract JoystickButton getShootButton();

  abstract JoystickButton getShootAmpButton();

  abstract JoystickButton getShootStopButton();

  abstract JoystickButton getPivotButtonOne();

  abstract JoystickButton getPivotButtonTwo();

  abstract JoystickButton getPivotButtonOff();

  abstract JoystickButton getFeederButton();

  abstract JoystickButton getFeederStopButton();

  abstract double getClimberAxis();

  abstract double getManualPivotAxis();

  abstract JoystickButton getLEDPatternOneButton();

  abstract JoystickButton getLEDPatternTwoButton();

  abstract JoystickButton getLEDPatternOffButton();

  private void registerPrototype() {
    if (ExampleConfig.Subsystems.PROTOTYPE_ENABLED) {
      Prototypes prototypes = Prototypes.getInstance();
      getShootStopButton().whileTrue(prototypes.runAny4Motors(-0.0, 0.0, 0.0, 0));
      getShootButton().whileTrue(prototypes.runAny4Motors(-0.30, 0.30, 0.0, 0));
      getFeederButton().whileTrue(prototypes.runAny4Motors(0.0, 0.0, -0.1, 0.0));
      getFeederStopButton().whileTrue(prototypes.runAny4Motors(0.0, 0.0, 0.1, 0.0));
    }
  }

  private void registerIntake() {
    if (ExampleConfig.Subsystems.INTAKE_ENABLED) {
      Intake intake = Intake.getInstance();

      getIntakeStopButton().onTrue(intake.setIntakeState(IntakeDirection.STOPPED));
      getIntakeButton().onTrue(intake.intakeUntilLoadedCommand());
      getIntakeReverseButton().onTrue(intake.setIntakeState(IntakeDirection.REVERSE));
    }
  }

  private void registerShamper() {
    if (ExampleConfig.Subsystems.SHAMPER_ENABLED) {
      Shamper shamper = Shamper.getInstance();

      // getShootButton().whileTrue(shamper.setFlywheelVelocityCommand(0.5));
      // getShootButton().whileFalse(shamper.stopFlywheelCommand());

      // getShootAmpButton().whileTrue(shamper.runFlywheelPower(0.2));
      // getShootAmpButton().whileTrue(shamper.stopFlywheel());
      // getFeederButton().onTrue(shamper.runFeederPower(0.9, false));
      // // getFeederButton().whileFalse(shamper.runFeederPower(0, false));
      // getFeederStopButton().onTrue(shamper.runFeederPower(0, false));
      // //getPivotButtonOne().onTrue(shamper.runPivot(10));
      // //getPivotButtonTwo().onTrue(shamper.runPivot(20));
      // getFeederButton().onTrue(shamper.runFeeder(1));
      // getFeederStopButton().onTrue(shamper.runFeeder(0.0));
      // shamper.setDefaultCommand(shamper.runPivotPower(() -> getClimberAxis()));
      // getPivotButtonOne().onTrue(shamper.runPivot(30.0));
      // getPivotButtonTwo().onTrue(shamper.runPivot(60.0));
      // getPivotButtonOff().onTrue(shamper.runPivot(0.0));
      // shamper.setDefaultCommand(shamper.runPivotPower(() -> getManualPivotAxis()));

    }
  }

  private void registerPivot() {
    if (ExampleConfig.Subsystems.PIVOT_ENABLED) {
      Pivot pivot = Pivot.getInstance();

      // getPivotButtonOne().onTrue(shamper.runPivot(10.0));
      // getPivotButtonTwo().onTrue(shamper.runPivot(20.0));
      // getPivotButtonOff().onTrue(shamper.runPivot(0.0));
    }
  }

  private void registerClimber() {
    if (ExampleConfig.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();
      climber.setDefaultCommand(climber.run(this::getClimberAxis));
    }
  }

  private void registerLEDs() {
    if (ExampleConfig.Subsystems.LEDS_ENABLED) {
      PWMLEDLights lights = PWMLEDLights.getInstance();
      getLEDPatternOffButton().onTrue(
          lights.setColorForSecondsCommand(3, BlinkinUtils.ColorPatterns.WHITE));
      getLEDPatternOneButton().onTrue(
          lights.setColorCommand(BlinkinUtils.ColorPatterns.SINELON_RAINBOW_PALETTE));
      getLEDPatternTwoButton().onTrue(
          lights.setColorCommand(BlinkinUtils.ColorPatterns.CP1_2_END_TO_END_BLEND));
    }
  }

  @Override
  public void registerCommands() {
    // registerPrototype();
    registerIntake();
    registerShamper();
    registerClimber();
    // registerLEDs();
  }
}
