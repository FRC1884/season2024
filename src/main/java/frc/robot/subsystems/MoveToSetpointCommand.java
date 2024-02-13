package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToSetpointCommand extends Command {

    private PIDController controller;
    private double kDt;
    private Consumer<Double> setMotorSpeed;
    private Supplier<Double> getPosition, getVelocity;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State current, goal;

    private double setpoint;

    public MoveToSetpointCommand(Consumer<Double> setMotorSpeed, Supplier<Double> getPosition, Supplier<Double> getVelocity, PIDController pid, double kDt, double tolerance, TrapezoidProfile.Constraints constraints, double setpoint) {
        this.setMotorSpeed = setMotorSpeed;
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        this.controller = pid;
        this.kDt = kDt;

        this.constraints = constraints;
        this.setpoint = setpoint;

        controller.setTolerance(tolerance);
    }

    @Override
    public void initialize() {
        goal = new TrapezoidProfile.State(setpoint, 0);
        current = new TrapezoidProfile.State(getPosition.get(), getVelocity.get());
        updateCurrent();
    }

    private TrapezoidProfile getProfile() {
        return new TrapezoidProfile(constraints);
    }

    private void updateCurrent() {
        var profile = getProfile();
        current = profile.calculate(kDt, current, goal);
    }

    @Override
    public void execute() {
        setMotorSpeed.accept(controller.calculate(getPosition.get(), current.position));
        updateCurrent();

        SmartDashboard.putNumber("profile enc", getPosition.get());
        SmartDashboard.putNumber("profile cur pos", current.position);
        SmartDashboard.putNumber("profile target", goal.position);
        SmartDashboard.putBoolean("profile fin", controller.atSetpoint());
        SmartDashboard.putNumber("profile err", controller.getPositionError());
       
        SmartDashboard.putNumber("profile tolerance", controller.getPositionTolerance());
    }
    
    @Override
    public boolean isFinished() {
        return (getProfile().isFinished(0)) && controller.atSetpoint();
    }
}
