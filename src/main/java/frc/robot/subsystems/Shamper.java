package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.FlywheelMap;

/**
 * This is the Catapul-- umm... Flywheel subsystem for the 2024 season. Throughout the season, add
 * everything shooting here. <br>
 * <br>
 * Think:
 *
 * <ul>
 *   <li>pitch control for adjusting the launch angle,
 *   <li>LUT-based power setpoints,
 *   <li>closed-loop control to regain rotational momentum quickly,
 *   <li>whatever else you'd like!
 * </ul>
 */
public class Shamper extends SubsystemBase {
  private static Shamper instance;

  public static Shamper getInstance() {
    if (instance == null) instance = new Shamper();
    return instance;
  }

  // Motor Controllers
  /*
   * leaderFlywheel: TOP FLYWHEEL
   * followerFlywheel: BOTTOM FLYWHEEL
   * leaderPIVOT: LEFT PIVOT
   * followerPivot: RIGHT PIVOT
   */
  private CANSparkBase leaderFlywheel = null, followerFlywheel = null;
  private CANSparkBase leaderPivot = null, followerPivot = null;
  private CANSparkBase feeder = null;

  private Shamper() {
    if(FlywheelMap.TOP_SHOOTER != -1)
      leaderFlywheel = new CANSparkFlex(FlywheelMap.TOP_SHOOTER, MotorType.kBrushless);
    if(FlywheelMap.BOTTOM_SHOOTER != -1)
      followerFlywheel = new CANSparkFlex(FlywheelMap.BOTTOM_SHOOTER, MotorType.kBrushless);
    if(FlywheelMap.LEFT_PIVOT != -1)
      leaderPivot = new CANSparkFlex(FlywheelMap.LEFT_PIVOT, MotorType.kBrushless);
    if(FlywheelMap.RIGHT_PIVOT != -1)
      followerPivot = new CANSparkFlex(FlywheelMap.RIGHT_PIVOT, MotorType.kBrushless);
    if(FlywheelMap.FEEDER != -1)
      feeder = new CANSparkFlex(FlywheelMap.FEEDER, MotorType.kBrushless);

    // This method is in case one of the motors needs to be inverted before setting them to follow
    leaderFlywheel.setInverted(false);
    followerFlywheel.follow(leaderFlywheel, true);

    leaderPivot.setInverted(false);
    followerPivot.follow(leaderPivot, true);

    feeder.setInverted(false);
  }

  public Command runFlywheel(double power) {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3);
    return new InstantCommand(() -> shootAtPower(sRL.calculate(power)), this);
    /*final double powerNorm = Math.min(power, 1.0);
    return new InstantCommand(
        () -> {
          leader_pidController.setReference(powerNorm, CANSparkMax.ControlType.kVelocity);
          follower_pidController.setReference(powerNorm, CANSparkMax.ControlType.kVelocity);
        });*/
  }

  public Command runPivot(double power) {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3);
    return new InstantCommand(() -> pivotAtPower(sRL.calculate(power)), this);
    /*final double powerNorm = Math.min(power, 1.0);
    return new InstantCommand(
        () -> {
          leader_pidController.setReference(powerNorm, CANSparkMax.ControlType.kVelocity);
          follower_pidController.setReference(powerNorm, CANSparkMax.ControlType.kVelocity);
        });*/
  }

  public Command runFeeder(double power) {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3);
    return new InstantCommand(() -> feed(sRL.calculate(power)), this);
    /*final double powerNorm = Math.min(power, 1.0);
    return new InstantCommand(
        () -> {
          leader_pidController.setReference(powerNorm, CANSparkMax.ControlType.kVelocity);
          follower_pidController.setReference(powerNorm, CANSparkMax.ControlType.kVelocity);
        });*/
  }

  private void shootAtPower(double power) {
    leaderFlywheel.set(leaderFlywheel.get() == 0.0 ? power : 0.0);
  }

  private void pivotAtPower(double power) {
    leaderPivot.set(leaderPivot.get() == 0.0 ? power : 0.0);
  }

  private void feed(double power) {
    feeder.set(feeder.get() == 0.0 ? power : 0.0);
  }
}
