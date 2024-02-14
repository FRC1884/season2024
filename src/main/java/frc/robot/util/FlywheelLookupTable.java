package frc.robot.util;

import static frc.robot.util.BlinkinUtils.ColorPatterns.valueOf;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class FlywheelLookupTable {
    private static FlywheelLookupTable instance;

    public static FlywheelLookupTable getInstance() {
        if (instance == null)
            instance = new FlywheelLookupTable();
        return instance;
    }

    private InterpolatingTreeMap<Double, Double> distanceToRPM, distanceToAngleSetpoint;

    // Distance (meters), rpm, angleSetpoint
    private double[][] lookupTable = {
            { 1, 1000, 85.2 },
            { 2, 1500, 80.5 }
    };

    private FlywheelLookupTable() {
        createShootMap(lookupTable);
    }

    private void createShootMap(double[][] table) {
        for (double[] t : table) {
            Double d = (t[0]);
            distanceToRPM.put(d, t[1]);
            distanceToAngleSetpoint.put(d, t[2]);
        }
    }

    public ActionSetpoint get(Double d) {
        ActionSetpoint values = new ActionSetpoint(distanceToRPM.get(d), distanceToAngleSetpoint.get(d));
        return values;
    }

    public void clearTables() {
        distanceToAngleSetpoint.clear();
        distanceToRPM.clear();
    }

}
