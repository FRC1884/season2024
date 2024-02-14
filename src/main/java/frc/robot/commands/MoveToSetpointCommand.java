package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MoveToSetpointCommand extends Command {

    private PIDController controller;
    private double kDt;
    private Consumer<Double> setMotorSpeed;
    private Supplier<Double> getPosition, getVelocity;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State current, goal;

    private double tolerance;
    private double setpoint;

    /**
     * Creates a command to move to a setpoint using a trapezoidal motion profile.
     * 
     * @param setMotorSpeed A consumer that sets the speed of a motor or other
     *                      mechanism
     * @param getPosition   A supplier that returns the current position of the
     *                      mechanism
     * @param getVelocity   A supplier that returns the current velocity of the
     *                      mechanism
     * @param pid           A set of PID Constants to use for the command
     * @param kDt           The time step to use for the profile
     * @param tolerance     A tolerance to use to determine when the mechanism is at
     *                      the setpoint
     * @param constraints   The constraints to use for the profile (max velocity and
     *                      acceleration)
     * @param setpoint      The setpoint to move to
     * @param name          The name of the mechanism for logging
     * @param requirements  usually this.
     */
    public MoveToSetpointCommand(Consumer<Double> setMotorSpeed, Supplier<Double> getPosition,
            Supplier<Double> getVelocity, PIDConstants pid, double kDt, double tolerance,
            TrapezoidProfile.Constraints constraints, double setpoint, String name, Subsystem... requirements) {

        this.setMotorSpeed = setMotorSpeed;
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        this.controller = new PIDController(pid.kP, pid.kI, pid.kD);
        this.kDt = kDt;
        this.constraints = constraints;
        this.setpoint = setpoint;
        this.tolerance = tolerance;

        controller.setTolerance(tolerance);

        addRequirements(requirements);

        var tab = Shuffleboard.getTab(name + " profile");

        tab.addDouble("enc", () -> getPosition.get());
        tab.addDouble("target cur pos", () -> current.position);
        tab.addDouble("target pos", () -> goal.position);
        tab.addDouble("target speed", () -> calculateSpeed());
        tab.addDouble("err", () -> controller.getPositionError());
        tab.addDouble("calculated error", () -> Math.abs(setpoint - getPosition.get()));

        tab.addBoolean("fin", () -> isDone() && controller.atSetpoint());
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

    private double calculateSpeed() {
        var speed = controller.calculate(getPosition.get(), current.position);

        return speed;
    }

    @Override
    public void execute() {
        setMotorSpeed.accept(calculateSpeed());
        updateCurrent();
    }

    private boolean isDone() {
        return Math.abs(setpoint - getPosition.get()) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        setMotorSpeed.accept(0.0);
    }

    @Override
    public boolean isFinished() {
        return (isDone()) && controller.atSetpoint();
    }
}
