package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap.PivotMap;

public class Pivot extends ProfiledPIDSubsystem {
    private static Pivot instance;

    public static Pivot getInstance() {
        if (instance == null)
            instance = new Pivot();
        return instance;
    }

    private SparkPIDController pivotPID;
    private CANSparkMax pivot;

    private Pivot() {
        super(
                new ProfiledPIDController(PivotMap.kP,
                        PivotMap.kI,
                        PivotMap.kD,
                        PivotMap.PROFILE_CONSTRAINTS));

        setName("Pivot");
        enable();
        pivot = new CANSparkMax(PivotMap.PIVOT_ID, MotorType.kBrushless);
        pivot.restoreFactoryDefaults();
        pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivot.burnFlash();

        pivotPID = pivot.getPIDController();
        var tab = Shuffleboard.getTab("Pivot");
        tab.add(this);
    }

    private void zeroPivot() {
        pivot.getEncoder().setPosition(0);
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var feedforward = 0; // pivotFeedforward.calculate(state.position, state.velocity);

        pivotPID.setReference(v + feedforward, CANSparkBase.ControlType.kVoltage);
    }

    @Override
    protected double getMeasurement() {
        return pivot.getEncoder().getPosition();
    }

    public boolean isAtGoal() {
        return getController().atGoal();
    }

    public void setPosition(double setpoint) {
        getController().setGoal(setpoint);
    }

    @Override
    public void periodic() {
        if (!isAtGoal()) {
            
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("goal", () -> getController().getGoal().position, this::setGoal);
        builder.addDoubleProperty("encoder", this::getMeasurement, (s) -> {
        });
        builder.addDoubleArrayProperty("pid", () -> new double[] { pivotPID.getD(), pivotPID.getI(), pivotPID.getD() },
                (pid) -> {
                    pivotPID.setP(pid[0]);
                    pivotPID.setI(pid[1]);
                    pivotPID.setD(pid[2]);
                });
    }
}