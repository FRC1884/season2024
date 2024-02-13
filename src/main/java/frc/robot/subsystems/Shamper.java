package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ShamperMap;
import frc.robot.util.SendableMotor;
import frc.robot.RobotMap.PIDMap;
import frc.robot.RobotMap.Pivot;

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
  private CANSparkBase leaderFlywheel, followerFlywheel;
  private CANSparkMax pivot;
  private CANSparkBase feeder;
  private SparkPIDController leaderFlywheel_PIDController, followerFlywheel_PIDController, pivot_PIDController;
  private TrapezoidProfile profile1;

  private SparkLimitSwitch pivotReverseLimitSwitch, pivotForwardLimitSwitch;

  // private RelativeEncoder pivotEncoder;

  private SendableMotor pivotSendable;

  private Shamper() {
    if (ShamperMap.TOP_SHOOTER != -1) {
      leaderFlywheel = new CANSparkFlex(ShamperMap.TOP_SHOOTER, MotorType.kBrushless);
      leaderFlywheel_PIDController = leaderFlywheel.getPIDController();
      leaderFlywheel_PIDController.setP(PIDMap.P);
      leaderFlywheel_PIDController.setI(PIDMap.I);
      leaderFlywheel_PIDController.setD(PIDMap.D);
    }
    if (ShamperMap.BOTTOM_SHOOTER != -1) {
      followerFlywheel = new CANSparkFlex(ShamperMap.BOTTOM_SHOOTER, MotorType.kBrushless);
      followerFlywheel_PIDController = followerFlywheel.getPIDController();
      followerFlywheel_PIDController.setP(PIDMap.P);
      followerFlywheel_PIDController.setI(PIDMap.I);
      followerFlywheel_PIDController.setD(PIDMap.D);
    }
    if (ShamperMap.PIVOT != -1){
      pivot = new CANSparkMax(ShamperMap.PIVOT, MotorType.kBrushless);
      pivot.restoreFactoryDefaults();
      pivot_PIDController = pivot.getPIDController();

      pivot.setInverted(true);

      // pivotEncoder =
      // pivot.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

      // System.out.println(pivotEncoder.getCountsPerRevolution());
      // System.out.println(pivotEncoder.getPosition());

      var tab = Shuffleboard.getTab("shamper");

      tab.addDouble("enc", () -> pivot.getEncoder().getPosition());

      pivotReverseLimitSwitch = pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      pivotReverseLimitSwitch.enableLimitSwitch(true);

      pivotForwardLimitSwitch = pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      pivotForwardLimitSwitch.enableLimitSwitch(true);

      pivot.burnFlash();

      tab.addBoolean("rev switch", () -> pivotReverseLimitSwitch.isPressed());
      tab.addBoolean("for switch", () -> pivotForwardLimitSwitch.isPressed());

      // pivot_PIDController.setFeedbackDevice();

      pivot_PIDController.setP(PIDMap.P * 20);
      pivot_PIDController.setI(PIDMap.I * 20);
      pivot_PIDController.setD(PIDMap.D * 20);
    }
    if (ShamperMap.FEEDER != -1)
      feeder = new CANSparkFlex(ShamperMap.FEEDER, MotorType.kBrushless);

    // This method is in case one of the motors needs to be inverted before setting
    // them to follow
    leaderFlywheel.setInverted(false);
    followerFlywheel.follow(leaderFlywheel, true);

    feeder.setInverted(false);

  }

  public double getSetpoint() {
    return leaderFlywheel.getEncoder().getPosition();
  }

  public Command runFlywheel() {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3, -0.3, leaderFlywheel.getEncoder().getVelocity());
    return new InstantCommand(
        () -> {
          leaderFlywheel.set(0.1);
          // leaderFlywheel.getPIDController().setReference(sRL.calculate(rpmToMetersPS(power)),
          // CANSparkBase.ControlType.kVelocity);
          // followerFlywheel.getPIDController().setReference(sRL.calculate(rpmToMetersPS(power)),
          // CANSparkBase.ControlType.kVelocity);
        });
  }

  public Command runFlywheelPower(double power){
    return new InstantCommand(() -> leaderFlywheel.set(power));
  }

  public Command stopFlywheel(){
    return new InstantCommand(() -> leaderFlywheel.set(0));
  }


  public Command runPivot(double setpoint) {
    return new MoveToSetpointCommand(speed -> {if (speed > 1) pivot.set(1.0); else if (speed < -1) pivot.set(-1.0); else pivot.set(speed);}, () -> pivot.getEncoder().getPosition(), () -> pivot.getEncoder().getVelocity(), Pivot.PID, Pivot.DT, Pivot.TOLERANCE, Pivot.PROFILE_CONSTRAINTS, setpoint, this);
  } 

  public Command runPivotPower(Supplier<Double> power) {
    return new InstantCommand(() -> {
      pivot.set(power.get());
    }, this);
  }

  public Command runFeeder(double power) {
    SlewRateLimiter sRL = new SlewRateLimiter(0.3);
    return new InstantCommand(
        () -> {
          feeder.set(sRL.calculate(power));
        });
  }

  public Command runFeederPower(double power, boolean placeHolder){
    return new InstantCommand(() -> feeder.set(power));
  }

  private static double rpmToMetersPS(double value) {
    return (value * 60) / (2 * (Math.PI) * ShamperMap.FLYWHEEL_RADIUS);
  }

  private void zeroPivot() {
    pivot.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    if(ShamperMap.PIVOT != -1){
    if (pivotReverseLimitSwitch.isPressed())
      zeroPivot();
    // SmartDashboard.putNumber("pivot enc", pivotEncoder.getPosition());
  }
  }
}