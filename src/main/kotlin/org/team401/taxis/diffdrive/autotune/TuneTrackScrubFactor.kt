package org.team401.taxis.diffdrive.autotune

import com.ctre.phoenix.motorcontrol.ControlMode
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.AngularDistanceUnit
import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureDegrees

/**
 * @author Cameron Earle
 * @version 9/10/18
 *
 * Tunes the track scrub factor.  Make sure no other threads are updating the drive
 *
 * Right turns increase angle
 */
class TuneTrackScrubFactor(val drivetrain: TankDrivetrain, turns: Int = 10, power: Double = 1.0, direction: Boolean = true): AutoStep() {
    private val turnRadians = turns * 2 * Math.PI
    private val turnPower = if (direction) power else -power

    override fun entry(currentTime: Double) {
        drivetrain.setYaw(AngularDistanceMeasureDegrees(0.0))
        drivetrain.left.setPosition(AngularDistanceMeasureCTREMagEncoder(0.0))
        drivetrain.right.setPosition(AngularDistanceMeasureCTREMagEncoder(0.0))
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        drivetrain.tank(ControlMode.PercentOutput, turnPower, -turnPower)
        val yaw = Math.abs(drivetrain.getYaw().toUnit(AngularDistanceUnit.Standard.RADIANS).value)
        if (yaw >= turnRadians) {
            val leftPos = drivetrain.left.getPosition().toLinearDistance(drivetrain.wheelRadius).toUnit(LinearDistanceUnit.Standard.INCHES).value
            val rightPos = drivetrain.right.getPosition().toLinearDistance(drivetrain.wheelRadius).toUnit(LinearDistanceUnit.Standard.INCHES).value
            val arcLength = (leftPos + rightPos) / 2

            val trackWidth = 2 * yaw / arcLength

            println("Empirical Trackwidth (Inches): $trackWidth")
            println("Track Scrub Factor: ${trackWidth / drivetrain.wheelbase.toUnit(LinearDistanceUnit.Standard.INCHES).value}")

            drivetrain.stop()
            return true
        }
        return false
    }

    override fun exit(currentTime: Double) {
    }

}