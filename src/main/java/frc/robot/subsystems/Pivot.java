package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap;
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
    // private SparkLimitSwitch forwardLimitSwitch, reverseLimitSwitch;
    private DigitalInput reverseLimitSwitch;
    private boolean shouldZeroPivot;
    // private RelativeEncoder pivotEncoder;

    private Pivot() {
        super(
                new ProfiledPIDController(PivotMap.kP,
                        PivotMap.kI,
                        PivotMap.kD,
                        PivotMap.PROFILE_CONSTRAINTS));

        setName("Pivot");
        pivot = new CANSparkMax(PivotMap.PIVOT_ID, MotorType.kBrushless);
        //pivot.restoreFactoryDefaults();
        pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivot.setSmartCurrentLimit(40);
        

        // pivotEncoder = pivot.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        pivotPID = pivot.getPIDController();

        // disables the integrator if |error| is too high, i.e. above the value of kIZone
        pivotPID.setIZone(RobotMap.PivotMap.kIZone);

        // forwardLimitSwitch = pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // forwardLimitSwitch.enableLimitSwitch(false);

        // reverseLimitSwitch = pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // reverseLimitSwitch.enableLimitSwitch(true);

        // reverseLimitSwitch = new DigitalInput(2);

        pivot.burnFlash();

        var tab = Shuffleboard.getTab("Pivot");
        tab.add(this);

        setGoal(0);
        enable();
    }

    private void zeroPivot(boolean zP) {
        if(zP)
        pivot.getEncoder().setPosition(0);
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var feedforward = 0;//pivotFeedforward.calculate(state.position, state.velocity);

        //pivotPID.setReference(v + feedforward, CANSparkBase.ControlType.kVoltage);
        pivot.setVoltage(v + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return pivot.getEncoder().getPosition();
    }

    public boolean isAtGoal() {
        return getController().atGoal();
    }

    public void setPosition(double setpoint) {
        if(setpoint<PivotMap.LOWER_SETPOINT_LIMIT && setpoint>PivotMap.UPPER_SETPOINT_LIMIT)
            getController().setGoal(setpoint);
    }

    public Command updatePosition(Supplier<Double> setpoint)
    {
        return new RunCommand(() -> setPosition(setpoint.get()), this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void setSpeed(double spd){
        pivot.set(spd);
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("goal", () -> getController().getSetpoint().position, (s) -> setPosition(s));
        //builder.addBooleanProperty("Forward limit", () -> forwardLimitSwitch.isPressed(), (s) -> {});
        // builder.addBooleanProperty("Reverse limit", () -> reverseLimitSwitch.get(), (s) -> {});
        builder.addDoubleProperty("encoder", this::getMeasurement, (s) -> {});
        builder.addDoubleProperty("kP", () -> m_controller.getP(), (s) -> m_controller.setP(s));
        builder.addDoubleProperty("kI", () -> m_controller.getI(), (s) -> m_controller.setI(s));
        builder.addDoubleProperty("kD", () -> m_controller.getD(), (s) -> m_controller.setD(s));
        builder.addBooleanProperty("Zero Pivot", () -> shouldZeroPivot, (b) -> zeroPivot(b));
        builder.addBooleanProperty("at goal", () -> isAtGoal(), null);
        builder.addDoubleProperty("integrator zone", () -> m_controller.getIZone(), (s) -> m_controller.setIZone(s));
        //builder.addDoubleProperty("Pivot Power", () -> pivot.get(), (s) -> setSpeed(s));
        builder.addDoubleProperty("target V", () -> getController().calculate(getMeasurement()), null);

    }
}