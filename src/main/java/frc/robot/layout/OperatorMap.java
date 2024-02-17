package frc.robot.layout;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PivotMap;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.Commands.IntakeUntilLoadedCommand;
import frc.robot.Commands.ShootSequenceCommand;
import frc.robot.core.util.controllers.BoardController;
import frc.robot.util.BlinkinUtils;
import frc.robot.util.FlywheelLookupTable;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getOuttakeButton();

  abstract JoystickButton getShootSpeakerButton();

  abstract JoystickButton getShootAmpButton();

  abstract JoystickButton getAmpAlignButton();

  abstract JoystickButton getClimbSequenceButton();

  abstract double getManualPivotAxis();

  abstract JoystickButton getArcButton();

  abstract JoystickButton getTrapButton();

  abstract JoystickButton getStageAlignButton();

  abstract JoystickButton getManualShootButton();

  abstract JoystickButton getAmplifyButton();

  abstract JoystickButton getCoopButton();

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
      //getFeederButton().whileTrue(prototypes.runAny4Motors(0.0, 0.0, -0.1, 0.0));

    }
  }

  private void registerIntake() {
    if (Config.Subsystems.Intake.INTAKE_ENABLED) {
      Intake intake = Intake.getInstance();
      Shooter shooter = Shooter.getInstance();
      // getIntakeButton().onTrue(intake.setIntakeState(Intake.IntakeDirection.FORWARD).andThen(
      //   new InstantCommand(() -> shooter.setFeederState(FeederDirection.FORWARD))
      // ).until(() -> shooter.isNoteLoaded()).andThen(
      //   intake.setIntakeState(Intake.IntakeDirection.STOPPED)
      // ));
     getOuttakeButton().onTrue(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeDirection.REVERSE), intake));
      

    }
  }

  private void registerShooter() {
    if (Config.Subsystems.SHOOTER_ENABLED) {
      Shooter shooter = Shooter.getInstance();
      Pivot pivot = Pivot.getInstance();
      FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
      Pose2d target = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
      PoseEstimator poseEstimator = PoseEstimator.getInstance();
      // pivot.setDefaultCommand(pivot.updatePosition(() -> lookupTable
      // .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngleSetpoint()));
      // shooter.setDefaultCommand(shooter.setFlywheelVelocityCommand(() -> lookupTable.get(
      //   poseEstimator.getDistanceToPose(target.getTranslation())).getRPM()));
    }
  }

  private void registerFeeder() {
    if(Config.Subsystems.FEEDER_ENABLED) {
      Feeder feeder = Feeder.getInstance();
      getShootSpeakerButton().onTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD)));
      getShootAmpButton().onTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD_SLOW)));
      getTrapButton().onTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD)));
    }
  }

  private void registerClimber() {
    if (Config.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();
      
    }
  }

  private void registerComplexCommands(){
    if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.Intake.INTAKE_ENABLED
        && Config.Subsystems.DRIVETRAIN_ENABLED) {
        Intake intake = Intake.getInstance(); 
        Shooter shooter = Shooter.getInstance();
         Pivot pivot = Pivot.getInstance();
      FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
      Pose2d target = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
      PoseEstimator poseEstimator = PoseEstimator.getInstance();
      // getShootSpeakerButton().onTrue(new ShootSequenceCommand());
      getIntakeButton().onTrue(new IntakeUntilLoadedCommand());
      getArcButton().whileTrue(new ParallelCommandGroup(pivot.updatePosition(() -> lookupTable
      .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngleSetpoint()), 
      shooter.setFlywheelVelocityCommand(() -> lookupTable.get(
        poseEstimator.getDistanceToPose(target.getTranslation())).getRPM())));
      getAmpAlignButton().onTrue(
        pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE).alongWith(
        shooter.setFlywheelVelocityCommand(() -> ShooterMap.AMP_SPEED)));
      getStageAlignButton().onTrue(
        pivot.updatePosition(() -> PivotMap.PIVOT_TRAP_ANGLE).alongWith(
        shooter.setFlywheelVelocityCommand(() -> ShooterMap.TRAP_SPEED)));
      getClimbSequenceButton().onTrue(
        new SequentialCommandGroup(
          Climber.getInstance().run(() -> 0.2),
          new WaitCommand(1),
          Climber.getInstance().run(() -> 0)
        )
      );
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
    registerIntake();
    registerFeeder();
    registerClimber();
    registerShooter();
    registerLEDs();
    registerComplexCommands();
  }
}
