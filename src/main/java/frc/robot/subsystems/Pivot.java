package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotMap.PivotMap;

public class Pivot extends ProfiledPIDSubsystem {
    private static Pivot instance;

    public static Pivot getInstance() {
        if (instance == null)
            instance = new Pivot();
        return instance;
    }

    private CANSparkMax pivot;
    private SparkPIDController pivotPid;
    private SparkLimitSwitch pivotReverseLimitSwitch, pivotForwardLimitSwitch;
    private ArmFeedforward pivotFeedforward = PivotMap.FEEDFORWARD;

    private Pivot() {
        super(
                new ProfiledPIDController(
                        PivotMap.PID.kP,
                        PivotMap.PID.kI,
                        PivotMap.PID.kD,
                        PivotMap.PROFILE_CONSTRAINTS),
                0);

        setName("Pivot");
        System.out.println("RUNNING PIVOT");

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

        pivotPid = pivot.getPIDController();

        
        var tab = Shuffleboard.getTab("pivot");
        tab.add(this);

        enable();
        setGoal(0);
    }

    // public Command moveToSetpointCommand(double setpoint) {
    // return new MoveToSetpointCommand(
    // speed -> pivot.set(speed),
    // () -> pivot.getEncoder().getPosition(),
    // () -> pivot.getEncoder().getVelocity(),
    //
    // PivotMap.PID,
    // PivotMap.DT,
    // PivotMap.TOLERANCE,
    // PivotMap.PROFILE_CONSTRAINTS,
    //
    // setpoint,
    //
    // "pivot",
    // this);
    // }
    //
    public Command setSpeedCommand(Supplier<Double> power) {
        return new InstantCommand(() -> {
            pivot.set(power.get());
        }, this);
    }

    private void zeroPivot() {
        pivot.getEncoder().setPosition(0);
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var feedforward = 0; // pivotFeedforward.calculate(state.position, state.velocity);

        pivotPid.setReference(v + feedforward, CANSparkBase.ControlType.kVelocity);
    }

    @Override
    protected double getMeasurement() {
        return pivot.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (pivotReverseLimitSwitch.isPressed()) {
            zeroPivot();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("goal", () -> getController().getGoal().position, this::setGoal);
        builder.addBooleanProperty("forward limit", () -> pivotForwardLimitSwitch.isPressed(), (s) -> {});
        builder.addBooleanProperty("rev limit", () -> pivotReverseLimitSwitch.isPressed(), (s) -> {});
        builder.addDoubleProperty("encoder", this::getMeasurement, (s) -> {});
        builder.addDoubleProperty("kp", () -> pivotPid.getP(), (s) -> pivotPid.setP(s));
        builder.addDoubleProperty("ki", () -> pivotPid.getI(), (s) -> pivotPid.setP(s));
        builder.addDoubleProperty("kd", () -> pivotPid.getD(), (s) -> pivotPid.setP(s));
        builder.addDoubleProperty("ks", () -> pivotFeedforward.ks,
                (s) -> pivotFeedforward = new ArmFeedforward(s, pivotFeedforward.kg,
                        pivotFeedforward.kv, pivotFeedforward.ka));
        builder.addDoubleProperty("kg", () -> pivotFeedforward.kg,
                (s) -> pivotFeedforward = new ArmFeedforward(pivotFeedforward.ks, s,
                        pivotFeedforward.kv, pivotFeedforward.ka));
        builder.addDoubleProperty("kv", () -> pivotFeedforward.kv,
                (s) -> pivotFeedforward = new ArmFeedforward(pivotFeedforward.ks, pivotFeedforward.kg,
                        s, pivotFeedforward.ka));
        builder.addDoubleProperty("ka", () -> pivotFeedforward.ka,
                (s) -> pivotFeedforward = new ArmFeedforward(pivotFeedforward.ks, pivotFeedforward.kg,
                        pivotFeedforward.kv, s));
        builder.addBooleanProperty("at goal",() -> 
        getController().atGoal(), null);
        builder.addBooleanProperty("is enabled",() -> 
        isEnabled(), null);

        builder.addDoubleProperty("target v", () -> getController().calculate(getMeasurement()), null);
    }
}