package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.core.util.drivers.TalonSRXFactory;
import java.util.LinkedHashMap;

public class ExampleElevator extends SubsystemBase {
  private static ExampleElevator instance;

  private static final int CurrentLimit = 20;

  public static ExampleElevator getInstance() {
    if (instance == null) instance = new ExampleElevator();
    return instance;
  }

  private static final double arbFF = 0.1238; // 0.1
  private TalonSRX motor;

  // Gains
  private static final double kP = 0.1;
  private static final double kI = 0.1;
  private static final double kD = 0.1;
  private static final double kF = 0.1;

  private static final int kMMacceleration = (1000); // sensorUnitsPer100msPerSec
  private static final int kMMvelocity = (1000); // sensorUnitsPer100ms

  private static final int kElevatorTolerance = 1000;

  public static Setpoint currentState;

  public static enum Setpoint {
    STATE_1,
    STATE_2,
    STATE_3
  }

  private static final LinkedHashMap<Setpoint, Integer> setpoints = new LinkedHashMap<>();

  private ExampleElevator() {
    setpoints.put(Setpoint.STATE_1, 100);
    setpoints.put(Setpoint.STATE_2, 200);
    setpoints.put(Setpoint.STATE_3, 300);

    motor = TalonSRXFactory.createDefaultTalon(RobotMap.ElevatorMap.master);

    motor.configPeakCurrentLimit(CurrentLimit);
    motor.configPeakCurrentDuration(0);
    motor.configContinuousCurrentLimit(CurrentLimit);

    // Config PID
    // TODO add timeout to constants
    motor.config_kP(0, kP, 0);
    motor.config_kI(0, kI, 0);
    motor.config_kD(0, kD, 0);
    motor.config_kF(0, kF, 0);

    motor.configMotionAcceleration(kMMacceleration);
    motor.configMotionCruiseVelocity(kMMvelocity);

    motor.setSensorPhase(true);
    motor.overrideLimitSwitchesEnable(false);

    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);

    motor.setNeutralMode(NeutralMode.Brake);
  }

  public boolean setPosition(int position) {
    // do checks here, return false if you can't move for any reason

    motor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, arbFF);

    return true;
  }

  public boolean setPosition(Setpoint setpoint) {
    if (setPosition(setpoints.get(setpoint))) {
      currentState = setpoint;
      return true;
    }

    return false;
  }

  public Command moveElevatorCommand(Setpoint pos) {
    return new InstantCommand(() -> setPosition(pos), this);
  }

  public double getError() {
    return (motor.getClosedLoopTarget() - motor.getSelectedSensorPosition());
  }

  public boolean isAtSetpoint() {
    return (Math.abs(getError()) < kElevatorTolerance);
  }
}
