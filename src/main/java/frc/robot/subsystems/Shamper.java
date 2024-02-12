package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ShamperMap;
import frc.robot.RobotMap.PIDMap;

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
  private CANSparkBase Pivot = null;
  private CANSparkBase feeder = null;
  private SparkPIDController leaderFlywheel_PIDController, followerFlywheel_PIDController, Pivot_PIDController;
  private TrapezoidProfile profile1;

  private Shamper() {
    if(ShamperMap.TOP_SHOOTER != -1){
      leaderFlywheel = new CANSparkFlex(ShamperMap.TOP_SHOOTER, MotorType.kBrushless);
      leaderFlywheel_PIDController = leaderFlywheel.getPIDController();}
    if(ShamperMap.BOTTOM_SHOOTER != -1){
      followerFlywheel = new CANSparkFlex(ShamperMap.BOTTOM_SHOOTER, MotorType.kBrushless);
      followerFlywheel_PIDController = followerFlywheel.getPIDController();}
    if(ShamperMap.PIVOT != -1){
      Pivot = new CANSparkFlex(ShamperMap.PIVOT, MotorType.kBrushless);
      Pivot_PIDController = leaderFlywheel.getPIDController();}
    if(ShamperMap.FEEDER != -1)
      feeder = new CANSparkFlex(ShamperMap.FEEDER, MotorType.kBrushless);

    // This method is in case one of the motors needs to be inverted before setting them to follow
    leaderFlywheel.setInverted(false);
    followerFlywheel.follow(leaderFlywheel, true);

    Pivot.setInverted(false);

    feeder.setInverted(false);
    
    profile1 = new TrapezoidProfile(new TrapezoidProfile.Constraints(50000.0,1.0));
    set();
    
  }

  public void set(){
        leaderFlywheel_PIDController.setP(PIDMap.P);
        leaderFlywheel_PIDController.setI(PIDMap.I);
        leaderFlywheel_PIDController.setD(PIDMap.D);
        followerFlywheel_PIDController.setP(PIDMap.P);
        followerFlywheel_PIDController.setI(PIDMap.I);
        followerFlywheel_PIDController.setD(PIDMap.D);
        Pivot_PIDController.setP(PIDMap.P);
        Pivot_PIDController.setI(PIDMap.I);
        Pivot_PIDController.setD(PIDMap.D);

    }

  public Command runFlywheel(double power) {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3, -0.3, leaderFlywheel.getEncoder().getVelocity());
    return new InstantCommand(
        () -> {
          leaderFlywheel_PIDController.setReference(sRL.calculate(rpmToMetersPS(power)), CANSparkBase.ControlType.kVelocity);
          followerFlywheel_PIDController.setReference(sRL.calculate(rpmToMetersPS(power)), CANSparkBase.ControlType.kVelocity);
        });
  }

  public Command runPivot(double setpoint) {
    TrapezoidProfile.State current = new TrapezoidProfile.State(leaderFlywheel.getEncoder().getPosition(),leaderFlywheel.getEncoder().getVelocity());
    TrapezoidProfile.State SetPoint = new TrapezoidProfile.State((setpoint/360.0)*4096.0, 0.0);
    return new InstantCommand(
        () -> {
          Pivot_PIDController.setReference(profile1.calculate(0,current, SetPoint).position, CANSparkBase.ControlType.kPosition);
        });
  }

  public Command runPivotPower(DoubleSupplier power) {
    return new RunCommand(()->{
      Pivot.set(Pivot.get() == 0.0 ? power.getAsDouble() : 0.0);
    }, this);
  }

  public Command runFeeder(double power) {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3);
    return new InstantCommand(
        () -> {
          feeder.set(sRL.calculate(power));
        });
      }

  private static double rpmToMetersPS(double value) {
    return (value*60)/(2 * (Math.PI)* ShamperMap.FLYWHEEL_RADIUS);
  }
}