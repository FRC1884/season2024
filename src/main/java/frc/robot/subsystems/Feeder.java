package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.FeederMap;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.subsystems.Feeder.NoteStatus;

public class Feeder extends SubsystemBase {
    private static Feeder instance;
    private boolean isDisabled = true;
    private AddressableLEDLights lights;

    public static Feeder getInstance() {
        if(instance == null) instance = new Feeder();
        return instance;
    }

    public static enum NoteStatus {
        EMPTY, LOADED
    }

    public static enum FeederDirection{
        FORWARD, FORWARD_SLOW, REVERSE, STOPPED
    }

    private NoteStatus status = NoteStatus.EMPTY;

    
    private CANSparkBase feeder;
    private SparkPIDController feedPID;
    private DigitalInput beamBreak;

    private double feedVel;

    private Feeder() {
        lights = AddressableLEDLights.getInstance();
        if (FeederMap.FEEDER != -1){
            feeder = new CANSparkFlex(FeederMap.FEEDER, MotorType.kBrushless);
            feeder.setIdleMode(IdleMode.kBrake);
            feedPID = feeder.getPIDController();
            feedPID.setP(FeederMap.FEEDER_PID.kP);
            feedPID.setI(FeederMap.FEEDER_PID.kI);
            feedPID.setD(FeederMap.FEEDER_PID.kD);
            feedPID.setFF(FeederMap.FEEDER_FF);

            feeder.setClosedLoopRampRate(FeederMap.FEEDER_RAMP_RATE);
            feeder.setInverted(false);
            feeder.burnFlash();
            
            beamBreak = new DigitalInput(FeederMap.BEAMBREAK);
        }
    }

    public boolean isNoteLoaded(){
        //Once tripped 
        return (status == NoteStatus.LOADED);
    }

    public void runFeeder(){
        feedVel = FeederMap.FEEDER_RPM;
    }

    public void setFeederState(FeederDirection direction){
        // switch (direction){
        //     case FORWARD:
        //     {feedVel = ShooterMap.FEEDER_RPM;
        //     System.out.println("Hello");
        //     }
        //     case REVERSE:
        //     feedVel = -ShooterMap.FEEDER_RPM;
        //     case STOPPED:
        //     feedVel = 0;
        // }

        if(direction == FeederDirection.FORWARD){
            feedVel = FeederMap.FEEDER_RPM;
        }
        else if (direction == FeederDirection.STOPPED){
            feedVel = 0;
        }
        else if (direction == FeederDirection.FORWARD_SLOW){
            feedVel = FeederMap.FEEDER_RPM/8;
        }
        else if (direction == FeederDirection.REVERSE){
            feedVel = FeederMap.FEEDER_RPM * -1;
        }

    }

    private void updateMotors() {
        if (feeder != null) {
            if (feedVel != 0) {
                feedPID.setReference(feedVel, ControlType.kVelocity);
            } else
                feeder.set(0);
        }
    }

    @Override
    public void periodic() {
        status = (beamBreak.get()) ? NoteStatus.EMPTY : NoteStatus.LOADED;
        updateMotors();
    }
    public void Amplify(boolean y){
        isDisabled = y;
        lights.setColorCommand(Color.kBlue);

    }
    public void Coop(boolean y){
        isDisabled = y;
        lights.setColorCommand(Color.kPurple);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("feeder velocity", () -> feedVel, (v) -> feedVel = v);
        builder.addDoubleProperty("real feeder velo", () -> feeder.getEncoder().getVelocity(), (d) -> {
        });
        builder.addBooleanProperty("BB", () -> beamBreak.get(), null);
        if(!isDisabled){
            if(beamBreak.get()){
                lights.setColorCommand(Color.kGreenYellow);
            }
            else lights.setColorCommand(Color.kRed);

        }

    }
}
