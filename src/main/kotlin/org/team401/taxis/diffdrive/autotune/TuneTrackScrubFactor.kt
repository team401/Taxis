package org.team401.taxis.diffdrive.autotune

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.AngularDistanceUnit

/**
 * @author Cameron Earle
 * @version 9/10/18
 *
 * Tunes the track scrub factor.  Make sure no other threads are updating the drive
 */
class TuneTrackScrubFactor(val drivetrain: TankDrivetrain, turns: Int = 10, direction: Double = 1.0): AutoStep() {
    private val turnDegrees = turns * 360.0

    override fun entry(currentTime: Double) {
    }

    override fun action(currentTime: Double, lastTime: Double) {
        val yaw = drivetrain.getYaw().toUnit(AngularDistanceUnit.Standard.DEGREES).value
        if (yaw >= turnDegrees) {

        }
    }

    override fun exit(currentTime: Double) {
    }

}