package frc.robot.subsystems.pivot;

public class SimPivot implements PivotHardware {
    private static SimPivot instance;

    public static SimPivot getInstance() {
        if (instance == null) instance = new SimPivot();
        return instance;
    }
    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public double getEncoderPosition() {
        return 0;
    }

    @Override
    public void zeroEncoder() {

    }
}
