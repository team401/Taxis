package org.team401.taxis.diffdrive

import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.trajectory.timing.TimingConstraint

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 * Aggregate class for the properties of a path.
 *
 * Units are inches per second
 */
data class Path(val reversed: Boolean,
                val waypoints: List<Pose2d>,
                val maxVelocity: Double,
                val maxAcceleration: Double,
                val maxVoltage: Double,
                val startVelocity: Double = 0.0,
                val endVelocity: Double = 0.0,
                val constraints: List<TimingConstraint<Pose2dWithCurvature>> = listOf())