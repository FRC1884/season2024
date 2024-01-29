package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.RobotMap.PrototypeMap.*;

/**
 * The prototyping subsystem. Currently, it has support for three SparkMAX controllers. This means
 * you can test with both NEO 550s and Vortexes. In the future, it might be worth going all the way
 * up to {@link edu.wpi.first.wpilibj.motorcontrol.MotorController} to facilitate Krakens and
 * Falcons, too. <br>
 * <br>
 * For now, this subsystem gives us the means to run mechanisms containing up to three motors with
 * dashboard-tunable parameters such as speed and feedback gains, allowing us to visualize their
 * actual performance during competition. <br>
 * <br>
 * <b>To tweak these parameters, you MUST run the project in Test Mode on the Driver Station!</b>
 */


public class PrototypeSubsystem extends SubsystemBase {
    private static PrototypeSubsystem instance;

    public static PrototypeSubsystem getInstance() {
        if(instance == null) instance = new PrototypeSubsystem();
        return instance;
    }
    private ArrayList<CANSparkBase> motors;

    private PrototypeSubsystem() {
        motors = new ArrayList<>();

        if(MOTOR_1_ENABLED) {
            motors.add(
                    MOTOR_1_CLASS.equals(CANSparkMax.class) ?
                            new CANSparkMax(MOTOR_ID_1, kBrushless) :
                            new CANSparkFlex(MOTOR_ID_1, kBrushless)
            );
            motors.get(0).getPIDController().setP(MOTOR_1_KP);
        }

        if(MOTOR_2_ENABLED) {
            motors.add(
                    MOTOR_2_CLASS.equals(CANSparkMax.class) ?
                            new CANSparkMax(MOTOR_ID_2, kBrushless) :
                            new CANSparkFlex(MOTOR_ID_2, kBrushless)
            );
            motors.get(1).getPIDController().setP(MOTOR_2_KP);
        }

        if(MOTOR_3_ENABLED) {
            motors.add(
                    MOTOR_3_CLASS.equals(CANSparkMax.class) ?
                            new CANSparkMax(MOTOR_ID_3, kBrushless) :
                            new CANSparkFlex(MOTOR_ID_3, kBrushless)
            );
            motors.get(2).getPIDController().setP(MOTOR_3_KP);
        }

        if(MOTOR_4_ENABLED) {
            motors.add(
                    MOTOR_4_CLASS.equals(CANSparkMax.class) ?
                            new CANSparkMax(MOTOR_ID_4, kBrushless) :
                            new CANSparkFlex(MOTOR_ID_4, kBrushless)
            );
            motors.get(3).getPIDController().setP(MOTOR_4_KP);
        }
    }

    protected void useOutput(double output, double setpoint) {
        for(CANSparkBase motor : motors) {
            motor.getPIDController().setReference(output, ControlType.kVelocity);
        }
    }

    protected double getMeasurement() {
        if(motors.size() == 1) return motors.get(0).getEncoder().getVelocity();
        else throw new RuntimeException("Disable the remaining motors for this function!");
    }

    public Command set(DoubleSupplier vel1, DoubleSupplier vel2, DoubleSupplier vel3, DoubleSupplier vel4) {
        return new RunCommand(() -> {
            if(MOTOR_1_ENABLED) motors.get(0).set(vel1.getAsDouble());
            if(MOTOR_2_ENABLED) motors.get(1).set(vel2.getAsDouble());
            if(MOTOR_3_ENABLED) motors.get(2).set(vel3.getAsDouble());
            if(MOTOR_4_ENABLED) motors.get(3).set(vel4.getAsDouble());
        }, this);
    }

    public Command runTo(DoubleSupplier vel) {
        return new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                42.0,
                                6.9)),
                setpointState -> useOutput(setpointState.velocity, setpointState.velocity),
                () -> new TrapezoidProfile.State(0, vel.getAsDouble()),
                TrapezoidProfile.State::new,
                this
        );
    }
}