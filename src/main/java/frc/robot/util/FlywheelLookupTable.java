package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class FlywheelLookupTable {

    private InterpolatingDoubleTreeMap distanceToRPM, distanceToAngleSetpoint;

    // Distance (meters), rpm, angleSetpoint
    private double[][] lookupTable;

    public FlywheelLookupTable(double[][] t) {
        lookupTable = t;

        distanceToRPM = new InterpolatingDoubleTreeMap();
        distanceToAngleSetpoint = new InterpolatingDoubleTreeMap();

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
