package org.team401.taxis.diffdrive.odometry

/**
 * @author Cameron Earle
 * @version 1/4/19
 */
interface OdometryProvider {
    fun getLatestXInches(): Double
    fun getLatestYInches(): Double
    fun getLatestThetaRadians(): Double
}