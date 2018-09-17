package org.team401.taxis.diffdrive.autotune

import com.ctre.phoenix.motorcontrol.ControlMode
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.AngularVelocityUnit
import org.snakeskin.units.TimeUnit
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 */
class CollectAngularData(val drivetrain: TankDrivetrain,
                         val power: Double,
                         val polarity: Double,
                         val accelerationTime: TimeMeasure): AutoStep() {
    data class DataPacket(val velocity: Double,
                          val voltage: Double,
                          val current: Double,
                          val angularVelocity: Double,
                          val angularAcceleration: Double)

    val data = arrayListOf<DataPacket>()

    private var startTime = 0.0
    private var prevTime = 0.0
    private var prevAngularPosition = 0.0
    private var prevAngularVelocity = -1.0

    override fun entry(currentTime: Double) {
        println("Collecting angular data")
        data.clear()
        prevAngularPosition = 0.0
        prevAngularVelocity = -1.0
        startTime = currentTime
        prevTime = startTime
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        if (currentTime - startTime > accelerationTime.toUnit(TimeUnit.Standard.SECONDS).value) {
            return true //All points collected
        }

        drivetrain.tank(ControlMode.PercentOutput, power * polarity, power * -polarity)

        val currentVelocity = (Math.abs(drivetrain.left.getVelocity().toUnit(AngularVelocityUnit.Standard.RADIANS_PER_SECOND).value) +
                               Math.abs(drivetrain.right.getVelocity().toUnit(AngularVelocityUnit.Standard.RADIANS_PER_SECOND).value)) / 2.0
        val avgCurrent = (drivetrain.left.master.outputCurrent + drivetrain.right.master.outputCurrent) / 2.0
        val angularPosition = drivetrain.imu.fusedHeading

        if (prevTime == startTime) {
            prevTime = currentTime
            prevAngularPosition = angularPosition
            return false //Initialize all variables
        }

        val angularVelocity = Math.abs(angularPosition - prevAngularPosition)

        if (prevAngularVelocity == -1.0) {
            prevTime = currentTime
            prevAngularPosition = angularPosition
            prevAngularVelocity = angularVelocity
            return false //Wait for at least one other velocity point for acceleration
        }

        val angularAcceleration = (angularVelocity - prevAngularVelocity) / (currentTime - prevTime)

        if (angularAcceleration < Util.kEpsilon) {
            prevTime = currentTime
            prevAngularPosition = angularPosition
            prevAngularVelocity = angularVelocity
            return false //Ignore points that are too small
        }

        data.add(DataPacket(
                currentVelocity,
                power * 12.0,
                avgCurrent,
                angularVelocity,
                angularAcceleration
        ))

        prevTime = currentTime
        prevAngularPosition = angularPosition
        prevAngularVelocity = angularVelocity
        return false
    }

    override fun exit(currentTime: Double) {
        drivetrain.stop()
        println("Done collecting angular data")
    }
}