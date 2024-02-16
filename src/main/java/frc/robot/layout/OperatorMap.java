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
import frc.robot.Config;
import frc.robot.Commands.ShootSequenceCommand;
import frc.robot.core.util.controllers.BoardController;
import frc.robot.util.BlinkinUtils;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getOuttakeButton();

  abstract JoystickButton getShootSpeakerButton();

  abstract JoystickButton getShootAmpButton();

  abstract JoystickButton getPivotButtonOne();

  abstract JoystickButton getPivotButtonTwo();

  abstract JoystickButton getFeederButton();

  abstract double getClimberAxis();

  abstract double getManualPivotAxis();

  abstract JoystickButton getLEDPatternOneButton();

  abstract JoystickButton getLEDPatternTwoButton();

  abstract JoystickButton getLEDPatternThreeButton();

  abstract JoystickButton getLEDPatternFourButton();

  abstract JoystickButton getLEDPatternFiveButton();

  abstract JoystickButton getLEDPatternOffButton();

  abstract double getLEDAxis1();

  abstract double getLEDAxis2();

  private void registerPrototype() {
    if (Config.Subsystems.PROTOTYPE_ENABLED) {
      Prototypes prototypes = Prototypes.getInstance();

      getShootSpeakerButton().whileTrue(prototypes.runAny4Motors(-0.30, 0.30, 0.0, 0));
      getFeederButton().whileTrue(prototypes.runAny4Motors(0.0, 0.0, -0.1, 0.0));

    }
  }

  private void registerIntake() {
    if (Config.Subsystems.Intake.INTAKE_ENABLED) {
      Intake intake = Intake.getInstance();
      getIntakeButton().onTrue(intake.setIntakeState(Intake.IntakeDirection.FORWARD));
      getOuttakeButton().onTrue(intake.setIntakeState(Intake.IntakeDirection.REVERSE));
      

    }
  }

  private void registerShooter() {
    if (Config.Subsystems.SHAMPER_ENABLED) {
      Shooter shooter = Shooter.getInstance();
      Pivot pivot = Pivot.getInstance();

    }
  }

  private void registerClimber() {
    if (Config.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();
      climber.setDefaultCommand(climber.run(this::getClimberAxis));
    }
  }

  private void registerShootSequence() {
    if (Config.Subsystems.SHAMPER_ENABLED && Config.Subsystems.Intake.INTAKE_ENABLED
        && Config.Subsystems.DRIVETRAIN_ENABLED) {
      getShootSpeakerButton().onTrue(new ShootSequenceCommand());
    }
  }

  private void registerLEDs() {
    if (Config.Subsystems.LEDS_ENABLED) {
      AddressableLEDLights lights = AddressableLEDLights.getInstance();
      // getLEDPatternOffButton().whileTrue(lights.disableCommand());
      // // getLEDPatternOneButton().onTrue(lights.setRedBlack());
      // getLEDPatternTwoButton().whileTrue(lights.setDecreasing());
      // getLEDPatternThreeButton().whileTrue(lights.setRainbow());
      // getLEDPatternFourButton().whileTrue(lights.setRedDarkRed());
      // getLEDPatternFiveButton().whileTrue(lights.checkBeam());
      lights.setDefaultCommand(lights.setValue(this::getLEDAxis1, this::getLEDAxis2));
    }
  }

  @Override
  public void registerCommands() {
    // registerPrototype();
    //registerIntake();
    //registerShooter();
    //registerShootSequence();

    // registerClimber();
    registerLEDs();
  }
}
