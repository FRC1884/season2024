package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShamperMap;

/**
 * This is the Catapul-- umm... Flywheel subsystem for the 2024 season.
 * Throughout the season, add
 * everything shooting here. <br>
 * <br>
 * Think:
 *
 * <ul>
 * <li>pitch control for adjusting the launch angle,
 * <li>LUT-based power setpoints,
 * <li>closed-loop control to regain rotational momentum quickly,
 * <li>whatever else you'd like!
 * </ul>
 */
public class Shamper extends SubsystemBase {
  private static Shamper instance;

  public static Shamper getInstance() {
    if (instance == null)
      instance = new Shamper();
    return instance;
  }

  // Motor Controllers
  /*
   * leaderFlywheel: TOP FLYWHEEL
   * followerFlywheel: BOTTOM FLYWHEEL
   * leaderPIVOT: LEFT PIVOT
   * followerPivot: RIGHT PIVOT
   */
  private CANSparkFlex top, bottom;
  private CANSparkFlex feeder;

  private SparkPIDController topPid, bottomPid, feederPid;

  private double topVelocity, bottomVelocity, feederVelocity;

  private Shamper() {
    setName("Shamper");

    if (ShamperMap.TOP_SHOOTER != -1) {
      top = new CANSparkFlex(ShamperMap.TOP_SHOOTER, MotorType.kBrushless);
      top.restoreFactoryDefaults();

      topPid = top.getPIDController();
      topPid.setP(ShamperMap.FLYWHEEL_PID.kP);
      topPid.setI(ShamperMap.FLYWHEEL_PID.kI);
      topPid.setD(ShamperMap.FLYWHEEL_PID.kD);
      topPid.setFF(ShamperMap.FLYWHEEL_FF);

      // takes that many seconds to go from 0 to full speed
      top.setClosedLoopRampRate(ShamperMap.FLYWHEEL_RAMP_RATE);
      top.setCANTimeout(0);

      top.burnFlash();
    }

    if (ShamperMap.BOTTOM_SHOOTER != -1) {
      bottom = new CANSparkFlex(ShamperMap.BOTTOM_SHOOTER, MotorType.kBrushless);
      bottom.restoreFactoryDefaults();

      bottom.setInverted(true);

      bottomPid = bottom.getPIDController();
      bottomPid.setP(ShamperMap.FLYWHEEL_PID.kP);
      bottomPid.setI(ShamperMap.FLYWHEEL_PID.kI);
      bottomPid.setD(ShamperMap.FLYWHEEL_PID.kD);
      bottomPid.setFF(ShamperMap.FLYWHEEL_FF);

      // takes that many seconds to go from 0 to full speed
      bottom.setClosedLoopRampRate(ShamperMap.FLYWHEEL_RAMP_RATE);
      bottom.setCANTimeout(0);

      bottom.burnFlash();
    }

    if (ShamperMap.FEEDER != -1) {
      feeder = new CANSparkFlex(ShamperMap.FEEDER, MotorType.kBrushless);
      feeder.restoreFactoryDefaults();

      feederPid = feeder.getPIDController();
      feederPid.setP(ShamperMap.FEEDER_PID.kP);
      feederPid.setI(ShamperMap.FEEDER_PID.kI);
      feederPid.setD(ShamperMap.FEEDER_PID.kD);
      feederPid.setFF(ShamperMap.FEEDER_FF);

      // takes that many seconds to go from 0 to full speed
      feeder.setClosedLoopRampRate(ShamperMap.FEEDER_RAMP_RATE);

      feeder.burnFlash();
    }

    var tab = Shuffleboard.getTab("Shamper");

    if (top != null) {
      // tab.add("top", top);
      // tab.add("top pid", topPid);
    }

    if (bottom != null) {
      // tab.add("bottom", bottom);
      // tab.add("bottom pid", bottomPid);
    }

    if (feeder != null) {
      // tab.add("feeder", feeder);
      // tab.add("feeder pid", feederPid);
    }

    tab.add("shamper", this);
  }

  // public Command setFlywheelVelocityCommand(double velocity) {
  //   return new InstantCommand(() -> {
  //     topVelocity = velocity;
  //     bottomVelocity = velocity;
  //   }, this);
  // }

  public Command setFlywheelVelocityCommand(Supplier<Double> velocity) {

    return setTopVelocityCommand(velocity).alongWith(setBotVelocityCommand(velocity));
  }

  public Command setTopVelocityCommand(Supplier<Double> v) {
    return new InstantCommand(
        () -> {
          topVelocity = v.get();
        });
  }
  
  public Command setBotVelocityCommand(Supplier<Double> v) {
    return new InstantCommand(
        () -> {
          bottomVelocity = v.get();
        }
    );
  }
  
  

  public Command stopFlywheelCommand() {
    return new InstantCommand(() -> {
      topVelocity = 0;
      bottomVelocity = 0;
    }, this);
  }

  public Command setFeederVelocityCommand(Supplier<Double> velocity) {
    return new InstantCommand(
        () -> {
          feederVelocity = velocity.get();
        });
  }

  public Command setShootVelocityCommand(Supplier<Double> vK, Supplier<Double> vF) {
    return new SequentialCommandGroup(
      setFlywheelVelocityCommand(vK),
      setFeederVelocityCommand(vF)
    );
  }

  public Command stopFeederCommand() {
    return new InstantCommand(() -> {
      feederVelocity = 0;
    });
  }

  @Override
  public void periodic() {
    //System.out.println(topVelocity);
    if (top != null) {
      if (topVelocity != 0) {
        topPid.setReference(topVelocity, ControlType.kVelocity);
      } else {
        top.set(0);
      }
    }

    if (bottom != null) {
      if (bottomVelocity != 0) {
        bottomPid.setReference(bottomVelocity, ControlType.kVelocity);
      } else {
        bottom.set(0);
      }
    }

    if (feeder != null) {
      if (feederVelocity != 0) {
        feederPid.setReference(feederVelocity, ControlType.kVelocity);
      } else {
        feeder.set(0);
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("top velocity", () -> topVelocity, (v) -> topVelocity = v);
    builder.addDoubleProperty("bottom velocity", () -> bottomVelocity, (v) -> bottomVelocity = v);
    builder.addDoubleProperty("feeder velocity", () -> feederVelocity, (v) -> feederVelocity = v);
    builder.addDoubleProperty("real top velo", () -> top.getEncoder().getVelocity(), (d)-> {});
    builder.addDoubleProperty("real bottom velo", () -> bottom.getEncoder().getVelocity(), (d) -> {});
    builder.addDoubleProperty("real feeder velo", () -> feeder.getEncoder().getVelocity(), (d)->{});

    builder.addDoubleProperty("top ff", () -> topPid.getFF(), (s) -> topPid.setFF(s));
    builder.addDoubleProperty("bottom ff", () -> bottomPid.getFF(), (s) -> bottomPid.setFF(s));
    builder.addDoubleProperty("feeder ff", () -> feederPid.getFF(), (s) -> feederPid.setFF(s));
  }
}