package frc.robot.subsystems;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.RobotMap;
import frc.robot.ExampleConfig.Subsystems;
import frc.robot.RobotMap.PrototypeMap;
import frc.robot.layout.OperatorMap;
import frc.robot.util.SendableMotor;

public class Prototypes extends SubsystemBase {
    private static Prototypes instance;

    public static Prototypes getInstance() {
        if(instance == null) instance = new Prototypes();
        return instance;
    }
    
    private CANSparkBase motor1, motor2, motor3, motor4;
    private SendableMotor motor1Sendable, motor2Sendable, motor3Sendable, motor4Sendable;
    private TrapezoidProfile profile1, profile2, profile3, profile4;
    private TrapezoidProfile.State trapezoidalSetpoint1, trapezoidalSetpoint2, trapezoidalSetpoint3, trapezoidalSetpoint4;
    // private double setpoint1;

    private double rampValue = 0.0; 
    
    private Prototypes() {
        if(Subsystems.PROTOTYPE_ENABLED) {
            profile1 = new TrapezoidProfile(new TrapezoidProfile.Constraints(50000.0,1.0));
            //motor1 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_1, MotorType.kBrushless); // TODO: Make sure that it is the right Motor
            //motor2 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
            //motor3 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
            // motor4 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_4, MotorType.kBrushless);

            if(PrototypeMap.LIVE_WINDOW_ENABLED) {
                motor1Sendable = new SendableMotor(motor1);
                motor2Sendable = new SendableMotor(motor2);
                motor3Sendable = new SendableMotor(motor3);
                //motor4Sendable = new SendableMotor(motor4);

                SendableRegistry.addLW(motor1Sendable, "Prototype", "Motor 1");
                SendableRegistry.addLW(motor2Sendable, "Prototype", "Motor 2");
                SendableRegistry.addLW(motor3Sendable, "Prototype", "Motor 3");
                // SendableRegistry.addLW(new SendableMotor(motor4), "Prototype", "Motor 4");
            }
        }
    }

    public void shufflePeriodic() {
        if(Subsystems.PROTOTYPE_ENABLED && PrototypeMap.LIVE_WINDOW_ENABLED) {
            if(motor1Sendable != null) motor1Sendable.control();
            if(motor2Sendable != null) motor2Sendable.control();
            if(motor3Sendable != null) motor3Sendable.control();
            if(motor4Sendable != null) motor4Sendable.control();
        }
    }

    public Command ramp(double rv) {
        return new RepeatCommand(
            new InstantCommand(() -> {this.rampValue += rv;})
        );
    }

    public double getRampValue() {
        return rampValue;
    }

    public Command run(double speed1, double speed2, double speed3, double speed4) {
        return new RunCommand(()->{
            if(Subsystems.PROTOTYPE_ENABLED && !RobotMap.PrototypeMap.LIVE_WINDOW_ENABLED) {
                if(motor1 != null) {
                    motor1.set(speed1 + getRampValue());
                }
                if(motor2 != null) {
                    motor2.set(speed2 - getRampValue());
                }
                if(motor3 != null) {
                    motor3.set(speed3);
                }
                if(motor4 != null) {
                    motor4.set(speed4);
                }
            }
            if(speed4 == 0) rampValue -= 0.01;
            if(speed4 == 1) rampValue += 0.01;
        }, this);
    }
}