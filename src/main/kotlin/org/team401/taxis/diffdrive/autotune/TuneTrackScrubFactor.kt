package org.team401.taxis.diffdrive.autotune

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRevolutions

/**
 * @author Cameron Earle
 * @version 9/10/18
 *
 * Tunes the track scrub factor.  Make sure no other threads are updating the drive
 *
 * Right turns increase angle
 */
class TuneTrackScrubFactor(val drivetrain: IYawSensoredDifferentialDrivetrain<ISensoredGearbox>, turns: Int = 10, power: Double = 1.0, direction: Boolean = true): AutoStep() {
    private val turnRadians = AngularDistanceMeasureRevolutions(turns.toDouble()).toRadians().value
    private val turnPower = if (direction) power else -power

    override fun entry(currentTime: Double) {
        drivetrain.setYaw(AngularDistanceMeasureDegrees(0.0).toRadians())
        drivetrain.left.setPosition(AngularDistanceMeasureRadians(0.0))
        drivetrain.right.setPosition(AngularDistanceMeasureRadians(0.0))
        Thread.sleep(1000)
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        drivetrain.tank(turnPower, -turnPower)
        val yaw = Math.abs(drivetrain.getYaw().value)
        if (yaw >= turnRadians) {
            val leftPos = Math.abs(drivetrain.left.getPosition().toLinearDistance(drivetrain.wheelRadius).value)
            val rightPos = Math.abs(drivetrain.right.getPosition().toLinearDistance(drivetrain.wheelRadius).value)
            val arcLength = (leftPos + rightPos) / 2

            val trackWidth = 2 * arcLength / yaw

            println("Empirical Trackwidth (Inches): $trackWidth")
            println("Track Scrub Factor: ${trackWidth / drivetrain.wheelbase.value}")

            drivetrain.stop()
            return true
        }
        return false
    }

    override fun exit(currentTime: Double) {
    }

}