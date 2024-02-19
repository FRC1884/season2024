package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.util.RunForSecondsCommand;

import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

import static frc.robot.util.BlinkinUtils.ColorPatterns.BLACK;

public class AddressableLEDLights extends SubsystemBase {
    private static AddressableLEDLights instance;
    public static AddressableLEDLights getInstance() {
        if(instance == null) instance = new AddressableLEDLights();
        return instance;
    }

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;
    private int currLED, direction, hue;
    private int z =1;
    private int x =0;
    // private DigitalInput feederSensor;
    private boolean c;

    private AddressableLEDLights() {
        m_led = new AddressableLED(RobotMap.LEDMap.BLINKIN_PWM_PORT);
        m_led.setLength(RobotMap.LEDMap.NUMBER_LEDS);
        m_ledBuffer = new AddressableLEDBuffer(RobotMap.LEDMap.NUMBER_LEDS);
        m_led.setData(m_ledBuffer);
        m_led.start();
        // feederSensor = new DigitalInput(4);
        // m_led_TWO = new AddressableLED(RobotMap.LEDMap.BLINKIN_PWM_PORT_TWO);
        // m_led_TWO.setLength(RobotMap.LEDMap.NUMBER_LEDS);
        // m_led_TWO.setData(m_ledBuffer);
        // m_led_TWO.start();
    }

    public Command setValue(DoubleSupplier confidence, DoubleSupplier confidence2) {
        return new RunCommand(
            ()->{
                for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(
                        i, 
                        120, 
                        (int) (127 + confidence2.getAsDouble() * 127),
                        (int) (127 + confidence.getAsDouble() * 127)
                    );
                }
                m_rainbowFirstPixelHue++;
                m_rainbowFirstPixelHue %= 180;
                m_led.setData(m_ledBuffer);
                m_led.start();
                // m_led_TWO.setData(m_ledBuffer);
                // m_led_TWO.start();
            }
        , this);
    }

    private void setColor(Color color) {
        for(int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(
                i, 
                (int) color.green * 255, 
                (int) color.red * 255, 
                (int) color.blue * 255
            );
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
        // m_led_TWO.setData(m_ledBuffer);
        // m_led_TWO.start();
        //IntStream.range(0, m_blinkinBuffer.getLength()).forEachOrdered(i -> m_blinkinBuffer.setLED(i, color));
    }

    public Command setRainbow() {
        return new RepeatCommand(new InstantCommand(() -> {
            // For every pixel
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
                // Set the value
                m_ledBuffer.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 1;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
            m_led.setData(m_ledBuffer);
            m_led.start();
            // m_led_TWO.setData(m_ledBuffer);
            // m_led_TWO.start();
        }, this));
    }
    public Command setDecreasing() {
        return new RepeatCommand(new InstantCommand(()->{
            for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, 0, 0, 0);
                // m_ledBuffer.setHSV(i+1, 0, 0, 0);
                // m_ledBuffer.setHSV(i+2, 0, 0, 0);
            }
            if(z>=270){z=0;currLED=0;direction=1;}
            // while(z>=x){
            //     m_ledBuffer.setHSV(currLED+x, hue, 255, 255); 
            //     x++;
            // }
            while(z>=x){
                if(c){
                m_ledBuffer.setHSV(currLED+x, 0, 255, 255); c=false;}
                else { m_ledBuffer.setHSV(currLED+x, 60, 255, 255);c=true;}
                x++;
            }

            // if(c){
            //     while(z>=x){
            //         m_ledBuffer.setHSV(currLED+x, 0, 255, 255); 
            //         x++;
            //     }
            //     c=false;
            // }
            // else {
            //     while(z>=x){
            //         m_ledBuffer.setHSV(currLED+x, 60, 255, 255); 
            //         x++;
            //     }
            //     c=true;
            // }
            x=0;
            hue+=+5;
            hue %= 180;
            currLED += direction;
            if(currLED == m_ledBuffer.getLength()-1-z) direction = -1;
            if(currLED == 0) direction = 1;
            if(currLED%3==0)z++;
            m_led.setData(m_ledBuffer);
            // m_led_TWO.setData(m_ledBuffer);
            
        }, this).andThen(new WaitCommand(0.0)));
    }

    public Command setRedBlack() {
        return new RepeatCommand(new InstantCommand(()->{
            for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, (currLED + i) % 2 == 0 ? Color.kRed: Color.kBlack);
            }
            currLED += 1;
            m_led.setData(m_ledBuffer);
            m_led.start();
            // m_led_TWO.setData(m_ledBuffer);
            // m_led_TWO.start();
        }).andThen(new WaitCommand(0.1)));
    }
    public Command setRedDarkRed() {
        return new RepeatCommand(new InstantCommand(() -> {
            if(c){
                setColor(Color.kRed);
                c=false;
            }
            else {
                setColor(Color.kYellow);
                c=true;
            }
        }, this));
    }
    public Command setColorCommand(Color color,boolean y) {
        if(y)return new RunCommand(() -> setColor(color), this);
        if(!y) return new RunCommand(()->setColor(Color.kBlack), this);
        return null;
    }

    public Command setToAllianceColorCommand() {
        return setColorCommand(DriverStation.getAlliance().get().equals(Alliance.Red) ? Color.kRed : Color.kBlue,true);
    }

    public Command setColorForSecondsCommand(double seconds, Color color) {
        return new RunForSecondsCommand(seconds, setColorCommand(color,true))
                .andThen(disableCommand());
    }

    public Command disableCommand() {
        return setColorCommand(Color.kBlack,true);
    }
    
    // public Command checkBeam() {
    //     return new RunCommand(()->{
    //     if(feederSensor.get()){
    //         setColor(Color.kGreenYellow);
    //     }
    //     else setColor(Color.kRed);
    //      },this);
    // }

    @Override
    public void periodic() {
        //System.out.println(m_ledBuffer.getLED(0));
    }
}