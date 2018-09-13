//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonSRXUtil {
    // Checks the specified error code for issues.
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}