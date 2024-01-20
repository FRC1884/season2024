package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Positionmotor extends SubsystemBase {
    private static final int deviceID = 1;
    private CANSparkMax m_motor;
    private SparkPIDController n_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
    private static Positionmotor instance;


    public static Positionmotor getInstance(){
        if (instance == null) instance = new Positionmotor();
        return instance;
        
    }
   
    private Positionmotor(){
        m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        n_pidController = m_motor.getPIDController();

        kP = 1.0;
        kI = 0.0;
        kD = 0.0;
        kIz = 0.0;
        kFF = 0.0;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxVel = 2000;
        minVel = 0;
        maxAcc = 1000;
        allowedErr = 0;

        configurePID(n_pidController);
   }

   private void configurePID(SparkPIDController pidController){
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMaxOutput, kMinOutput);
    pidController.setSmartMotionMaxVelocity(maxVel, 0);
    pidController.setSmartMotionMaxAccel(maxAcc, 0);
    pidController.setSmartMotionMinOutputVelocity(minVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, 0);
    pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

   }

   private void moveToSetPoint(double setpoint){
    n_pidController.setReference(setpoint, ControlType.kSmartMotion);
   }


   public Command runMotorToSetpoint(){
    return new InstantCommand(() -> moveToSetPoint(10));
   }



}
