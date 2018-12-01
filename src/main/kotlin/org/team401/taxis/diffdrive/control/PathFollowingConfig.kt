package org.team401.taxis.diffdrive.control

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasure
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasure

/**
 * @author Cameron Earle
 * @version 12/1/2018
 *
 */
data class PathFollowingConfig(val maxErrorX: LinearDistanceMeasure,
                               val maxErrorY: LinearDistanceMeasure,
                               val maxErrorTheta: AngularDistanceMeasure)