package org.team401.taxis.diffdrive.odometry

import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 1/4/19
 */
interface DriveDataProvider {
    fun getLeftPositionInches(): Double
    fun getRightPositionInches(): Double
    fun getLeftVelocityIps(): Double
    fun getRightVelocityIps(): Double
    fun getGyroHeading(): Rotation2d
}