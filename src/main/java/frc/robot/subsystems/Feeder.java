package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.FeederMap;

public class Feeder extends SubsystemBase {
    private static Feeder instance;
    private boolean isDisabled = false;
    private ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder Tab");
    private GenericEntry hasNoteEntry = feederTab.add("Has Note", false).getEntry();

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
        if (FeederMap.FEEDER != -1){

            setName("Feeder");
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

            var tab = Shuffleboard.getTab("Feeder");

            tab.add(this);
            
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
            feedVel = FeederMap.FEEDER_RPM_SLOW;
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
        if (status == NoteStatus.LOADED){
            hasNoteEntry.setBoolean(true);
        }
        else{
            hasNoteEntry.setBoolean(false);
        }
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("feeder velocity", () -> feedVel, (v) -> feedVel = v);
        builder.addDoubleProperty("real feeder velo", () -> feeder.getEncoder().getVelocity(), (d) -> {
        });
        builder.addBooleanProperty("HasNote", () -> isNoteLoaded(), null);
       

    }
}
