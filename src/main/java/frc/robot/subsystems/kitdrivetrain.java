package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.kitDriveMap;

public class kitdrivetrain extends SubsystemBase {
    private static kitdrivetrain instance;

    public static kitdrivetrain getInstance() {
        if (instance == null)
            instance = new kitdrivetrain();
        return instance;
    }
    
    private DifferentialDrive m_Drive;
    
    private CANSparkMax leftFront;
    private CANSparkMax leftBack;
    private CANSparkMax rightFront;
    private CANSparkMax rightBack;


    //constructor 
    private kitdrivetrain(){
        leftFront = new CANSparkMax(kitDriveMap.LF, MotorType.kBrushless);
        leftBack = new CANSparkMax(kitDriveMap.LB, MotorType.kBrushless);

        rightFront = new CANSparkMax(kitDriveMap.RF, MotorType.kBrushless);
        rightBack = new CANSparkMax(kitDriveMap.RB, MotorType.kBrushless);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        rightFront.setInverted(true);

        m_Drive = new DifferentialDrive(leftFront, rightFront);
    }
    public void drive(double speed, double rotation){
        m_Drive.arcadeDrive(speed, rotation);

    }

    public Command driveCommand(Supplier<Double> spd, Supplier<Double> rot){
        return new RunCommand(() -> drive(spd.get(), rot.get()), this);
    
    }
}