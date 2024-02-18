package frc.robot.util;

import static frc.robot.util.BlinkinUtils.ColorPatterns.valueOf;

import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class FlywheelLookupTable {
    private static FlywheelLookupTable instance;

    public static FlywheelLookupTable getInstance() {
        if (instance == null)
            instance = new FlywheelLookupTable();
        return instance;
    }

    private InterpolatingDoubleTreeMap distanceToFlywheelVelocity, distanceToAngleSetpoint, distanceToFeederVelocity;


    // Distance (meters), shooter velocity, angleSetpoint, feeder velocity
    private double[][] lookupTable = {
            // { 0.815, 2400, -115, 0},
            { 1.185, 3500, 46, 4000},
            { 1.61, 4000, 40, 4000},
            { 2.1, 4500, 36, 4000 },
            { 2.65, 4800, 32, 4000},
            { 3.14, 5100, 30, 4000},
            { 3.6, 5200, 28, 4000}
            // { 3.315, 4, -19, 0},
            // { 3.815, 4, -10, 0},
            // { 4.315, 4, -4, 0}
    };

    private FlywheelLookupTable() {
        distanceToFlywheelVelocity = new InterpolatingDoubleTreeMap();
        distanceToAngleSetpoint = new InterpolatingDoubleTreeMap();
        distanceToFeederVelocity = new InterpolatingDoubleTreeMap();

        createShootMap(lookupTable);
    }

    private void createShootMap(double[][] table) {
        for (double[] t : table) {
            Double d = (t[0]);
            distanceToFlywheelVelocity.put(d, t[1]);
            distanceToAngleSetpoint.put(d, t[2]);
            distanceToFeederVelocity.put(d, t[3]);
        }
    }

    public ActionSetpoint get(Double d) {
        ActionSetpoint values = new ActionSetpoint(distanceToFlywheelVelocity.get(d), distanceToAngleSetpoint.get(d), distanceToFeederVelocity.get(d));
        return values;
    }

    public void clearTables() {
        distanceToAngleSetpoint.clear();
        distanceToFlywheelVelocity.clear();
    }

}
