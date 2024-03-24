package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Config;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PivotMap;

public class Pivot extends ProfiledPIDSubsystem {
    private static Pivot instance;

    public static Pivot getInstance() {
        if (instance == null) instance = new Pivot();
        return instance;
    }

    private Optional<CANSparkMax> pivot;

    private Pivot() {
        super(new ProfiledPIDController(PivotMap.kP, PivotMap.kI, PivotMap.kD, PivotMap.PROFILE_CONSTRAINTS));

        if (Config.Subsystems.PIVOT_ENABLED) {
            pivot = Optional.of(new CANSparkMax(PivotMap.PIVOT_ID, MotorType.kBrushless));

            var motor = pivot.get();

            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            motor.setSmartCurrentLimit(40);

            // disables the integrator if |error| is too high, i.e. above the value of kIZone

            // TODO: test if the soft limits work at all
            motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) PivotMap.UPPER_SETPOINT_LIMIT);
            motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) PivotMap.LOWER_SETPOINT_LIMIT);

            motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

            var tab = Shuffleboard.getTab("Pivot");

            tab.add(this);



            // forwardLimitSwitch = pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
            // forwardLimitSwitch.enableLimitSwitch(false);

            // reverseLimitSwitch = pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
            // reverseLimitSwitch.enableLimitSwitch(true);

            // reverseLimitSwitch = new DigitalInput(2);

            motor.burnFlash();
        } else {
            pivot = Optional.empty();
        }

        m_controller.setIZone(PivotMap.kIZone);
        m_controller.setTolerance(PivotMap.POSITION_TOLERANCE, PivotMap.VELOCITY_TOLERANCE);

        setGoal(0);
        enable();
    }

    private void zeroPivot() {
        pivot.ifPresent(canSparkMax -> canSparkMax.getEncoder().setPosition(0));
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var feedforward = 0;//pivotFeedforward.calculate(state.position, state.velocity);

        //pivotPID.setReference(v + feedforward, CANSparkBase.ControlType.kVoltage);
        pivot.ifPresent(canSparkMax -> canSparkMax.setVoltage(v + feedforward));
    }

    @Override
    protected double getMeasurement() {
        return pivot.map(canSparkMax -> canSparkMax.getEncoder().getPosition()).orElse(0.0);
    }

    public boolean isAtGoal() {
        return getController().atGoal();
    }

    public void setPosition(double setpoint) {
        if (setpoint < PivotMap.LOWER_SETPOINT_LIMIT && setpoint > PivotMap.UPPER_SETPOINT_LIMIT)
            getController().setGoal(setpoint);
    }

    public Command updatePosition(Supplier<Double> setpoint) {
        return new RunCommand(() -> setPosition(setpoint.get()), this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("goal", () -> getController().getSetpoint().position, this::setPosition);
        // builder.addBooleanProperty("Forward limit", () -> forwardLimitSwitch.isPressed(), (s) -> {});
        // builder.addBooleanProperty("Reverse limit", () -> reverseLimitSwitch.get(), (s) -> {});
        builder.addDoubleProperty("encoder", this::getMeasurement, (s) -> {
        });

        builder.addDoubleProperty("kP", m_controller::getP, m_controller::setP);
        builder.addDoubleProperty("kI", m_controller::getI, m_controller::setI);
        builder.addDoubleProperty("kD", m_controller::getD, m_controller::setD);
        builder.addDoubleProperty("kIZone", m_controller::getIZone, m_controller::setIZone);

        builder.addDoubleProperty("max velo", () -> m_controller.getConstraints().maxVelocity,
                (v) -> m_controller.setConstraints(new TrapezoidProfile.Constraints(v, m_controller.getConstraints().maxAcceleration)));
        builder.addDoubleProperty("max accel", () -> m_controller.getConstraints().maxAcceleration,
                (v) -> m_controller.setConstraints(new TrapezoidProfile.Constraints(m_controller.getConstraints().maxVelocity, v)));

        builder.addDoubleProperty("forward limit", () -> pivot.map(canSparkMax -> canSparkMax.getSoftLimit(CANSparkBase.SoftLimitDirection.kForward)).orElse(0.0), (s) -> {
            pivot.ifPresent(canSparkMax -> canSparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) s));
        });
        builder.addDoubleProperty("reverse limit", () -> pivot.map(canSparkMax -> canSparkMax.getSoftLimit(CANSparkBase.SoftLimitDirection.kReverse)).orElse(0.0), (s) -> {
            pivot.ifPresent(canSparkMax -> canSparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) s));
        });

        builder.addBooleanProperty("Zero Pivot", () -> false, (b) -> {
            if (b) zeroPivot();
        });

        builder.addBooleanProperty("at goal", this::isAtGoal, null);
        builder.addDoubleProperty("target V", () -> getController().calculate(getMeasurement()), null);
    }
}