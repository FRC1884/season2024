package frc.robot.subsystems;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private SparkPIDController PIDController1, PIDController2, PIDController3, PIDController4;
    // TrapezoidProfiling not needing yet keep for now.
    // private TrapezoidProfile profile1, profile2, profile3, profile4;
    // private TrapezoidProfile.State trapezoidalSetpoint1, trapezoidalSetpoint2, trapezoidalSetpoint3, trapezoidalSetpoint4;

    private SlewRateLimiter sRL1,sRL2;
    // private double setpoint1;
    
    private Prototypes() {
        if(Subsystems.PROTOTYPE_ENABLED) {
            
            // profile1 = new TrapezoidProfile(new TrapezoidProfile.Constraints(50000.0,1.0));
            motor1 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_1, MotorType.kBrushless); // TODO: Make sure that it is the right Motor
            // motor2 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
            // motor3 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
            // motor4 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_4, MotorType.kBrushless);
            PIDController1 = motor1.getPIDController();
            //PIDController2 = motor2.getPIDController();
            // PIDController1.setP(PIDMap.P);
            // PIDController1.setI(PIDMap.I);
            // PIDController1.setD(PIDMap.D);
            // PIDController2.setP(PIDMap.P);
            // PIDController2.setI(PIDMap.I);
            // PIDController2.setD(PIDMap.D);
            // PIDController3.setP(PIDMap.P);
            // PIDController3.setI(PIDMap.I);
            // PIDController3.setD(PIDMap.D);
            // PIDController4.setP(PIDMap.P);
            // PIDController4.setI(PIDMap.I);
            // PIDController4.setD(PIDMap.D);

            if(PrototypeMap.LIVE_WINDOW_ENABLED) {
                motor1Sendable = new SendableMotor(motor1);
                // motor2Sendable = new SendableMotor(motor2);
                //motor3Sendable = new SendableMotor(motor3);
                //motor4Sendable = new SendableMotor(motor4);

                SendableRegistry.addLW(motor1Sendable, "Prototype", "Motor 1");
                // SendableRegistry.addLW(motor2Sendable, "Prototype", "Motor 2");
                //SendableRegistry.addLW(motor3Sendable, "Prototype", "Motor 3");
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
    // to be deleted don't now, only for testing if we dont have subsystem for it.
    public Command runAny4Motors(double speed1, double speed2, double speed3, double speed4) {
        return new RunCommand(()->{
            sRL1 = new SlewRateLimiter(0.7,-0.7,speed1);
            sRL2 = new SlewRateLimiter(0.7,-0.7,speed2);
            if(Subsystems.PROTOTYPE_ENABLED && !RobotMap.PrototypeMap.LIVE_WINDOW_ENABLED) {
                if(motor1 != null) {
                    if(speed1!= 0.0)
                    motor1.set(sRL1.calculate(speed1));
                    // PIDController1
                    //     .setReference((speed1*60)/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS), 
                    //  ControlType.kVelocity);
                     else motor1.set(0.0);
                }
                if(motor2 != null) {
                    if(speed2!= 0.0)
                    motor2.set(sRL2.calculate(speed2));
                    // PIDController2
                    //     .setReference((speed2*60)/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS), 
                    //  ControlType.kVelocity);
                     else motor2.set(0.0);
                }
                if(motor3 != null) {
                    if(speed3!= 0.0)
                    motor3.set(speed3);
                    //PIDController3
                    //     .setReference((speed3*60)/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS), 
                    //  ControlType.kVelocity);
                     else motor3.set(0.0);
                }
                if(motor4 != null) {
                    if(speed4!= 0.0)PIDController4
                        .setReference((speed4*60)/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS), 
                     ControlType.kVelocity);
                     else motor4.set(0.0);
                }
            }
        }, this);
    }
}