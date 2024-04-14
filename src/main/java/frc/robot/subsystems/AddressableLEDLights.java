package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Config;
import frc.robot.RobotMap.LEDMap;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.RobotMap.LEDMap.NUMBER_LEDS;

public class AddressableLEDLights extends SubsystemBase {
  private static AddressableLEDLights instance;
  public static AddressableLEDLights getInstance() {
    if(instance == null) instance = new AddressableLEDLights();
    return instance;
  }

//    public enum LEDState {
//        NO_NOTE,
//        HAS_NOTE,
//        AMPLIFY,
//        COOP;
//    }

//    private LEDState state = LEDState.NO_NOTE;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue = 0;
  private int value = 255, direction = -1;
  private int flagCurrentPixel = 3, flagDirection = 1;

  private static final double PERM_X_ERROR = 0.02, PERM_Y_ERROR = 0.02, PERM_ROT_ERROR = 0.5;

  private AddressableLEDLights() {
    m_led = new AddressableLED(LEDMap.BLINKIN_PWM_PORT);
    m_led.setLength(NUMBER_LEDS);
    m_ledBuffer = new AddressableLEDBuffer(NUMBER_LEDS);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

//    public LEDState getState() {
//        return state;
//    }
//
//    private Command setNoteStatusState(BooleanSupplier beamBroken) {
//        return Commands.runOnce(()-> {
//            if (beamBroken.getAsBoolean()) this.state = LEDState.HAS_NOTE;
//            else this.state = LEDState.NO_NOTE;
//        });
//    }
//
//    public Command toggleAmplifyState(BooleanSupplier beamBroken) {
//        return getState() == LEDState.AMPLIFY
//                ? setNoteStatusState(beamBroken)
//                : Commands.runOnce(() -> this.state = LEDState.AMPLIFY);
//    }
//
//    public Command toggleCoopState(BooleanSupplier beamBroken) {
//        return getState() == LEDState.COOP
//                ? setNoteStatusState(beamBroken)
//                : Commands.runOnce(() -> this.state = LEDState.COOP);
//    }
//
//    public Command useState(Supplier<LEDState> state) {
//        return Commands.select(
//                Map.of(
//                        LEDState.NO_NOTE, setColorCommand(Color.kRed),
//                        LEDState.HAS_NOTE, setBlinking(Color.kGreen, Color.kBlack, 6.0),
//                        LEDState.AMPLIFY, getAmplifyPattern(),
//                        LEDState.COOP, getCoOpPattern()
//                ),
//                this::getState
//        ).repeatedly();
//    }

  private void setColor(Color color) {
    for(int i = 0; i < NUMBER_LEDS; i++) {
      m_ledBuffer.setRGB(
        i,
        (int) color.green * 255,
        (int) color.red * 255,
        (int) color.blue * 255
      );
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  private void setColorLeft(Color color) {
    for(int i = 0; i < NUMBER_LEDS/2; i++) {
      m_ledBuffer.setRGB(
        i,
        (int) color.green * 255,
        (int) color.red * 255,
        (int) color.blue * 255
      );
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  private void setColorRight(Color color) {
    for(int i = NUMBER_LEDS/2; i < NUMBER_LEDS; i++) {
      m_ledBuffer.setRGB(
        i,
        (int) color.green * 255,
        (int) color.red * 255,
        (int) color.blue * 255
      );
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public Command setNoteStatusCommand(BooleanSupplier hasNote) {
    return (Commands.either(
      setColorCommand(Color.kRed),
      setColorCommand(Color.kGreenYellow),

      () -> hasNote.getAsBoolean()
    )).repeatedly();
  }

  private Command setRainbowCommand() {
    return Commands.runOnce(() -> {
      // For every pixel
      for (var i = 0; i < NUMBER_LEDS; i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / NUMBER_LEDS)) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 1;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(m_ledBuffer);
      m_led.start();
    }, this).repeatedly();
  }

    public Command getAmplifyPatternCommand() {
        return setPhaseInOutCommand(() -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

  public Command getCoOpPatternCommand() {
    return setRainbowCommand()
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command setAlingmentNoteStatusCommand(BooleanSupplier beamBroken) {
    return Commands.either(
      setBlinkingCommand(Color.kGreenYellow, Color.kBlack, 10.0),
      setBlinkingCommand(Color.kRed, Color.kBlack, 18.0),
      beamBroken
    );
  } 

  public Command setBlinkingCommand(Color onColor, Color offColor, double frequency) {
    return Commands.sequence(
      setColorCommand(offColor),
      Commands.waitSeconds(1.0 / frequency),
      setColorCommand(onColor),
      Commands.waitSeconds(1.0 / frequency)
    );
  }

  public Command setBlinkingLeftCommand(Color onColor, Color offColor, double frequency) {
    return Commands.repeatingSequence(
      setColorLeftCommand(onColor).withTimeout(1.0 / frequency),
      setColorLeftCommand(offColor).withTimeout(1.0 / frequency));
  }

  public Command setBlinkingRightCommand(Color onColor, Color offColor, double frequency) {
    return Commands.repeatingSequence(
      setColorRightCommand(onColor).withTimeout(1.0 / frequency),
      setColorRightCommand(offColor).withTimeout(1.0 / frequency));
  }

  private Command setPhaseInOutCommand(BooleanSupplier isRed) {
    return Commands.runOnce(() -> {
      for(int i = 0; i < NUMBER_LEDS; i++) {
        m_ledBuffer.setHSV(
          i,
          isRed.getAsBoolean() ? 240 : 120,
          0,
          value
        );
      }

      value += 10 * direction;

      if(value <= 0) {
        value = 0;
        direction = 1;
      }
      if(value >= 255) {
        value = 255;

        direction = -1;
      }

      m_led.setData(m_ledBuffer);
      m_led.start();
    }, this).repeatedly().beforeStarting(() -> value = 255);
  }

  public Command setColorCommand(Color color) {
    return Commands.runOnce(() -> setColor(color), this);
  }

  public Command setColorLeftCommand(Color color) {
    return Commands.run(() -> setColor(color), this);
  }

  public Command setColorRightCommand(Color color) {
    return Commands.run(() -> setColor(color), this);
  }

  // ping pongs BBWRWBB across the strip @kon
  public Command setGBFlagCommand() {
    return this.runOnce(() -> {
      for(int i = flagCurrentPixel - 3; i < flagCurrentPixel - 2; i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
        m_ledBuffer.setRGB(i+5, 0, 255, 0);
      }
      m_ledBuffer.setRGB(flagCurrentPixel - 1, 255, 255, 255);
      m_ledBuffer.setRGB(flagCurrentPixel, 255, 0, 0);
      m_ledBuffer.setRGB(flagCurrentPixel + 1, 255, 255, 255);

      flagCurrentPixel += flagDirection;

      if(flagCurrentPixel == NUMBER_LEDS - 4) flagDirection = -1;
      if(flagCurrentPixel == 3) flagDirection = 1;

      m_led.setData(m_ledBuffer);
      m_led.start();
    }).repeatedly().beforeStarting(()-> flagCurrentPixel = 3);
  }

  public Command alignmentHelperCommand(Supplier<Pose2d> current, Supplier<Pose2d> desired) {
    return Commands.run(
      () -> {
        double translationError = current.get().getTranslation().getDistance(desired.get().getTranslation());
        double rotationError = current.get().getRotation().minus(desired.get().getRotation()).getDegrees();

        Pose2d poseError = current.get().relativeTo(desired.get());

        // x = front-to-back, y = side-to-side, rot = positive-ccw
        // NOT ABSOLUTE! use the signs for info on direction
        double xError = poseError.getX();
        double yError = poseError.getY();
        double rotError = poseError.getRotation().getDegrees();

        Commands.either
          (
            Commands.repeatingSequence(
              setColorCommand(Color.kRed).until(() -> xError > -0.02),
              setColorCommand(Color.kWhite).until(() -> xError < 0.02),
              setColorLeftCommand(Color.kBlue).until(() -> yError > -0.02),
              setColorRightCommand(Color.kBlue).until(() -> yError < 0.02),
              setBlinkingLeftCommand(Color.kOrange, Color.kBlack, 2.0).until(() -> rotError > -0.02),
              setBlinkingRightCommand(Color.kOrange, Color.kBlack, 2.0).until(() -> rotError < 0.02)
            ),
          setColorCommand(Color.kGreenYellow),
          // accept if currently within 2cm and 0.5deg of the desired pose
          () -> translationError < Math.hypot(PERM_X_ERROR, PERM_Y_ERROR)
          && rotationError < PERM_ROT_ERROR
        );
      }
    )
      .ignoringDisable(true);
  }

}