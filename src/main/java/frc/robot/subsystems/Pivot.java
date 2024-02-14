package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PivotMap;
import frc.robot.commands.MoveToSetpointCommand;

public class Pivot extends SubsystemBase {
  private static Pivot instance;

  public static Pivot getInstance() {
    if (instance == null)
      instance = new Pivot();
    return instance;
  }

  private CANSparkMax pivot;
  private SparkLimitSwitch pivotReverseLimitSwitch, pivotForwardLimitSwitch;

  private Pivot() {
    setName("Pivot");

    pivot = new CANSparkMax(PivotMap.PIVOT, MotorType.kBrushless);
    pivot.restoreFactoryDefaults();

    pivot.setInverted(true);

    // BUG if we switch back to the alternate encoder then use this:
    // pivot.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    // pivot_PIDController.setFeedbackDevice();

    pivotReverseLimitSwitch = pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    pivotReverseLimitSwitch.enableLimitSwitch(true);

    pivotForwardLimitSwitch = pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    pivotForwardLimitSwitch.enableLimitSwitch(true);

    pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);

    pivot.burnFlash();

    var tab = Shuffleboard.getTab("pivot");

    tab.add("motor", pivot);
    tab.addDouble("enc", () -> pivot.getEncoder().getPosition());
    tab.addBoolean("rev switch", () -> pivotReverseLimitSwitch.isPressed());
    tab.addBoolean("for switch", () -> pivotForwardLimitSwitch.isPressed());

  }

  public Command moveToSetpointCommand(double setpoint) {
    return new MoveToSetpointCommand(
        speed -> pivot.set(speed),
        () -> pivot.getEncoder().getPosition(),
        () -> pivot.getEncoder().getVelocity(),

        PivotMap.PID,
        PivotMap.DT,
        PivotMap.TOLERANCE,
        PivotMap.PROFILE_CONSTRAINTS,

        setpoint,

        "pivot",
        this);
  }

  public Command setSpeedCommand(Supplier<Double> power) {
    return new InstantCommand(() -> {
      pivot.set(power.get());
    }, this);
  }

  private void zeroPivot() {
    pivot.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    if (pivotReverseLimitSwitch.isPressed()) {
      zeroPivot();
    }
  }
}