package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap.ClimberMap;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Climber extends ProfiledPIDSubsystem {
    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null)
            instance = new Climber();
        return instance;
    }

    private SparkPIDController climberPID;
    private CANSparkMax leader, follower;
    private boolean shouldZeroClimber;

    private Climber() {
        super(
          new ProfiledPIDController(
            ClimberMap.PID.kP,
            ClimberMap.PID.kI,
            ClimberMap.PID.kD,
            ClimberMap.PROFILE_CONSTRAINTS));

        setName("Climber");
        leader = new CANSparkMax(ClimberMap.LEADER, MotorType.kBrushless);
        follower = new CANSparkMax(ClimberMap.FOLLOWER, MotorType.kBrushless);
        leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leader.setSmartCurrentLimit(40);
        follower.setSmartCurrentLimit(40);

        climberPID = leader.getPIDController();

        leader.setSoftLimit(SoftLimitDirection.kReverse, 2);
        leader.setSoftLimit(SoftLimitDirection.kForward, 80);
        follower.setSoftLimit(SoftLimitDirection.kReverse, 2);
        follower.setSoftLimit(SoftLimitDirection.kForward, 80);

        follower.follow(leader);

        leader.setInverted(true);
        follower.setInverted(false);

        leader.burnFlash();
        follower.burnFlash();

        var tab = Shuffleboard.getTab("Climber");
        tab.add(this);

        setGoal(0);
        enable();
    }

    private void zeroClimber(boolean zC) {
        if(zC)
            leader.getEncoder().setPosition(0);
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var feedforward = 0;

        leader.setVoltage(v + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return leader.getEncoder().getPosition();
    }

    public boolean isAtGoal() {
        return getController().atGoal();
    }

    public void setPosition(double setpoint) {
        if(setpoint>=ClimberMap.LOWER_SETPOINT_LIMIT && setpoint<=ClimberMap.UPPER_SETPOINT_LIMIT)
            getController().setGoal(setpoint);
    }

    public Command updatePosition(Supplier<Double> setpoint)
    {
        return new RunCommand(() -> setPosition(setpoint.get()), this);
    }

    private DoubleSupplier getPosition() {
        return leader.getEncoder()::getPosition;
    }

    public void setSpeed(double spd){
        leader.set(spd);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("goal", () -> getController().getSetpoint().position, (s) -> setPosition(s));
        builder.addDoubleProperty("encoder", this::getMeasurement, (s) -> {});
        builder.addDoubleProperty("kP", () -> m_controller.getP(), (s) -> m_controller.setP(s));
        builder.addDoubleProperty("kI", () -> m_controller.getI(), (s) -> m_controller.setI(s));
        builder.addDoubleProperty("kD", () -> m_controller.getD(), (s) -> m_controller.setD(s));
        builder.addBooleanProperty("Zero Climber", () -> shouldZeroClimber, (b) -> zeroClimber(b));
        builder.addBooleanProperty("at goal", () -> isAtGoal(), null);
        builder.addDoubleProperty("integrator zone", () -> m_controller.getIZone(), (s) -> m_controller.setIZone(s));
        builder.addDoubleProperty("target V", () -> getController().calculate(getMeasurement()), null);

    }
}