package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.RobotMap.PivotMap;

import java.util.function.Supplier;


public class Pivot extends ProfiledPIDSubsystem {
    private static Pivot instance;



    public static Pivot getInstance() {
        if (instance == null) instance = new Pivot();
        return instance;
    }

    private final PivotHardware hardware = switch (Config.Subsystems.PIVOT_HARDWARE_TYPE) {
        case SINGLE_ACTUATOR -> SingleActuatorPivot.getInstance();
        case DUAL_ACTUATOR -> DualActuatorPivot.getInstance();
        case NONE -> SimPivot.getInstance();
    };

    private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, PivotMap.kG, PivotMap.kV, 0);

    private Pivot() {
        super(new ProfiledPIDController(PivotMap.kP, PivotMap.kI, PivotMap.kD, PivotMap.PROFILE_CONSTRAINTS));

        m_controller.setIZone(PivotMap.kIZone);
        m_controller.setTolerance(PivotMap.POSITION_TOLERANCE, PivotMap.VELOCITY_TOLERANCE);

        var tab = Shuffleboard.getTab("Pivot");
        tab.add(this);

        setGoal(PivotMap.PIVOT_RESTING_ANGLE);
        enable();
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var feedforward = pivotFeedforward.calculate(state.position, state.velocity);

        hardware.setVoltage(v + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return hardware.getEncoderPosition();
    }

    public boolean isAtGoal() {
        return getController().atGoal();
    }

    public void setPosition(double setpoint) {
        if ((Config.Subsystems.PIVOT_HARDWARE_TYPE == PivotHardware.PivotHardwareType.DUAL_ACTUATOR
                && setpoint > PivotMap.LOWER_SETPOINT_LIMIT && setpoint < PivotMap.UPPER_SETPOINT_LIMIT) ||
                (Config.Subsystems.PIVOT_HARDWARE_TYPE == PivotHardware.PivotHardwareType.SINGLE_ACTUATOR
                        && setpoint < PivotMap.LOWER_SETPOINT_LIMIT && setpoint > PivotMap.UPPER_SETPOINT_LIMIT))
            getController().setGoal(setpoint);
    }

    public Command setPositionCommand(Supplier<Double> setpoint) {
        return new RunCommand(() -> setPosition(setpoint.get()));
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("goal", () -> getController().getSetpoint().position, this::setPosition);
        builder.addDoubleProperty("encoder", this::getMeasurement, (s) -> {
        });

        builder.addDoubleProperty("kP", m_controller::getP, m_controller::setP);
        builder.addDoubleProperty("kI", m_controller::getI, m_controller::setI);
        builder.addDoubleProperty("kD", m_controller::getD, m_controller::setD);
        builder.addDoubleProperty("kIZone", m_controller::getIZone, m_controller::setIZone);

        builder.addDoubleProperty("kG", () -> pivotFeedforward.kg, (v) -> pivotFeedforward = new ArmFeedforward(0, v, pivotFeedforward.kv, 0));
        builder.addDoubleProperty("kV", () -> pivotFeedforward.kv, (v) -> pivotFeedforward = new ArmFeedforward(0, pivotFeedforward.kg, v, 0));

        builder.addDoubleProperty("max velo", () -> m_controller.getConstraints().maxVelocity,
                (v) -> m_controller.setConstraints(new TrapezoidProfile.Constraints(v, m_controller.getConstraints().maxAcceleration)));
        builder.addDoubleProperty("max accel", () -> m_controller.getConstraints().maxAcceleration,
                (v) -> m_controller.setConstraints(new TrapezoidProfile.Constraints(m_controller.getConstraints().maxVelocity, v)));

        builder.addBooleanProperty("Zero Pivot", () -> false, (b) -> {
            if (b) hardware.zeroEncoder();
        });

        builder.addBooleanProperty("at goal", this::isAtGoal, null);
        builder.addDoubleProperty("target V", () -> getController().calculate(getMeasurement()), null);
    }
}
