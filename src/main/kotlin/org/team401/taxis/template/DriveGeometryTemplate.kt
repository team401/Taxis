package org.team401.taxis.template

import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasure

/**
 * @author Cameron Earle
 * @version 8/30/2018
 *
 * TOOD Move this to SnakeSkin
 */
interface DriveGeometryTemplate {
    val wheelDiameter: LinearDistanceMeasure

    val wheelbase: LinearDistanceMeasure
}