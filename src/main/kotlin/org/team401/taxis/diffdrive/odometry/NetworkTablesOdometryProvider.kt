package org.team401.taxis.diffdrive.odometry

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/**
 * @author Cameron Earle
 * @version 1/4/19
 */
class NetworkTablesOdometryProvider(val tableKey: String = "SmartDashbaord",
                                    val xKey: String = "robot_x",
                                    val yKey: String = "robot_y",
                                    val thetaKey: String = "robot_theta"): OdometryProvider {
    private val table = NetworkTableInstance.getDefault().getTable(tableKey)
    private val xEntry = table.getEntry(xKey)
    private val yEntry = table.getEntry(yKey)
    private val thetaEntry = table.getEntry(thetaKey)

    override fun getLatestXInches(): Double {
        return xEntry.getDouble(0.0)
    }

    override fun getLatestYInches(): Double {
        return yEntry.getDouble(0.0)
    }

    override fun getLatestThetaRadians(): Double {
        return thetaEntry.getDouble(0.0)
    }
}