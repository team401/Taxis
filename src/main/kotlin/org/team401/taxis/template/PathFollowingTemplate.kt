package org.team401.taxis.template

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasure
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasure

/**
 * @author Cameron Earle
 * @version 8/30/2018
 *
 */
interface PathFollowingTemplate {
    val maxErrorX: LinearDistanceMeasure
    val maxErrorY: LinearDistanceMeasure
    val maxErrorTheta: AngularDistanceMeasure
}