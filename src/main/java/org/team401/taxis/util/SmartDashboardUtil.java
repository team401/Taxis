//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small class filled with static util methods for the SmartDashboard.
 */
public class SmartDashboardUtil {

    public static void deletePersistentKeys() {
        SmartDashboard smartDashboard = new SmartDashboard();
        for (String key : smartDashboard.getKeys()) {
            if (smartDashboard.isPersistent(key)) {
                smartDashboard.delete(key);
            }
        }
    }

    public static void deleteAllKeys() {
        SmartDashboard smartDashboard = new SmartDashboard();
        for (String key : smartDashboard.getKeys()) {
            smartDashboard.delete(key);
        }
    }
}
