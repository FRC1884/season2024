package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeMap;
import frc.robot.RobotMap.PIDMap;
import frc.robot.RobotMap.PrototypeMap;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private SparkPIDController PIDController1;
    private SparkPIDController PIDController2;
    

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
        
    }
    public void set(){
        PIDController1.setP(PIDMap.P);
        PIDController1.setI(PIDMap.I);
        PIDController1.setD(PIDMap.D);
        PIDController2.setP(PIDMap.P);
        PIDController2.setI(PIDMap.I);
        PIDController2.setD(PIDMap.D);
    }

    private CANSparkMax motor1, motor2;
    private double MOTOR_SPEED_1 = 30; //TODO: fix value
    private double MOTOR_SPEED_2 = -30;// TODO: fix value
    

    private Intake() {
        motor1 = new CANSparkMax(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
        motor2 = new CANSparkMax(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
   
        PIDController1 = motor1.getPIDController();
        PIDController2 = motor2.getPIDController();
        set();
        
    }

    private void run(double speed1, double speed2){
        motor1.getPIDController()
          .setReference((speed2*60)/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS), 
            ControlType.kVelocity);
        // motor1.set(speed1);
        // motor2.set(speed2);
        motor2.getPIDController()
          .setReference((speed1*60)/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS), 
            ControlType.kVelocity);
        System.out.println((motor1.getEncoder().getVelocity()*2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS)/60);
        System.out.println((motor2.getEncoder().getVelocity()*2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS)/60);
    }

    public Command runCommand() {
        return new InstantCommand(()->{
            run(MOTOR_SPEED_1, MOTOR_SPEED_2);
        }, this);
    }

    public Command stopCommand(){
        return new InstantCommand(
            () -> run(0, 0)
        ); 
    }

    
}
