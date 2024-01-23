package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.RobotMap;

import static frc.robot.RobotMap.PrototypeMap.MOTOR_ID_1;

/**
 * The prototyping subsystem. Currently, it has support for three SparkMAX controllers.
 * This means you can test with both NEO 550s and Vortexes. In the future,
 * it might be worth going all the way up to
 * {@link edu.wpi.first.wpilibj.motorcontrol.MotorController}
 * to facilitate Krakens and Falcons, too.
 * <br><br>
 * For now, this subsystem gives us the means to run mechanisms containing up to three motors
 * with dashboard-tunable parameters such as speed and feedback gains, allowing us to visualize
 * their actual performance during competition.
 * <br><br>
 * <b>To tweak these parameters, you MUST run the project in Test Mode on the Driver Station!</b>
 */

/*
public class PrototypeSubsystem extends PIDSubsystem {
    private static PrototypeSubsystem instance;

    public static PrototypeSubsystem getInstance() {
        if(instance == null) instance = new PrototypeSubsystem();
        return instance;
    }

    private CANSparkFlex sparkFlex;

    private PrototypeSubsystem() {
        super(new PIDController(
                RobotMap.PrototypeMap.MOTOR_1_KP,
                RobotMap.PrototypeMap.MOTOR_1_KI,
                RobotMap.PrototypeMap.MOTOR_1_KD));
        sparkFlex = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_1, CANSparkLowLevel.MotorType.kBrushless);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        sparkFlex.getPIDController().setReference(output,
                CANSparkBase.ControlType.kVelocity);
    }

    @Override
    protected double getMeasurement() {
        return sparkFlex.getEncoder().getVelocity();
    }

    public Command runTo(double vel) {
        return new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                3.0,
                                3.0)),
                setpointState -> useOutput(setpointState.velocity, setpointState.velocity),
                () -> new TrapezoidProfile.State(0,vel),
                TrapezoidProfile.State::new,
                this
        );
    }
}
*/