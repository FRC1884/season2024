package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.RobotMap;
import frc.robot.util.SendableMotor;

public class PrototypingFinal extends SubsystemBase {
    private static PrototypingFinal instance;

    public static PrototypingFinal getInstance() {
        if(instance == null) instance = new PrototypingFinal();
        return instance;
    }

    private CANSparkBase motor1, motor2, motor3, motor4;
    private SendableMotor motor1Sendable, motor2Sendable, motor3Sendable, motor4Sendable;
    // private double setpoint1;
    
    private PrototypingFinal() {
        motor1 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_1, MotorType.kBrushless); // TODO: Make sure that it is the right Motor
        motor2 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
        motor3 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
        // motor4 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_4, MotorType.kBrushless);

        motor1Sendable = new SendableMotor(motor1);
        motor2Sendable = new SendableMotor(motor2);
        motor3Sendable = new SendableMotor(motor3);
        //motor4Sendable = new SendableMotor(motor4);


        SendableRegistry.addLW(motor1Sendable, "Prototype", "Motor 1");
        SendableRegistry.addLW(motor2Sendable, "Prototype", "Motor 2");
        SendableRegistry.addLW(motor3Sendable, "Prototype", "Motor 3");
        // SendableRegistry.addLW(new SendableMotor(motor4), "Prototype", "Motor 4");
    }

    public void vPeriodic() {
        // if(motor1Sendable.m_setpoint != setpoint1)
        //     new TrapezoidProfile(new TrapezoidProfile.Constraints(600, 60)).calculate(0, null, null);

        if(!motor1Sendable.closedLoopEnabled) {
            if (motor1Sendable.openLoopEnabled) motor1.set(motor1Sendable.m_speed);
            else motor1.set(0.0);
        }

        else {
            if (motor1Sendable.openLoopEnabled) motor1.getPIDController()
            .setReference(motor1Sendable.m_setpoint, ControlType.kVelocity);
            else motor1.set(0.0);
        }
        
        // setpoint1 = motor1Sendable.m_setpoint;

        if(!motor2Sendable.closedLoopEnabled) {
            if (motor2Sendable.openLoopEnabled) motor1.set(motor1Sendable.m_speed);
            else motor2.set(0.0);
        }

        else {
            if (motor2Sendable.openLoopEnabled) motor1.getPIDController()
            .setReference(motor2Sendable.m_setpoint, ControlType.kVelocity);
            else motor2.set(0.0);
        }
        if(!motor3Sendable.closedLoopEnabled) {
            if (motor3Sendable.openLoopEnabled) motor1.set(motor1Sendable.m_speed);
            else motor3.set(0.0);
        }

        else {
            if (motor3Sendable.openLoopEnabled) motor1.getPIDController()
            .setReference(motor3Sendable.m_setpoint, ControlType.kVelocity);
            else motor3.set(0.0);
        }

    // if(!motor4Sendable.closedLoopEnabled) {
    //   if (motor4Sendable.openLoopEnabled) motor1.set(motor1Sendable.m_speed);
    //   else motor4.set(0.0);
    // }

    // else {
    //   if (motor4Sendable.openLoopEnabled) motor1.getPIDController()
    //   .setReference(motor4Sendable.m_setpoint, ControlType.kVelocity);
    //   else motor4.set(0.0);
    // }
    }
}
