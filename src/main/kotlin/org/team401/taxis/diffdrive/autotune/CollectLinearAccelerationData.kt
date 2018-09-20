package org.team401.taxis.diffdrive.autotune

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.TimeUnit
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.physics.DriveCharacterization
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 9/19/2018
 *
 */
class CollectLinearAccelerationData(val drive: TankDrivetrain,
                                    val power: Double,
                                    val time: TimeMeasure): AutoStep() {
    private val timeSeconds = time.toUnit(TimeUnit.Standard.SECONDS).value

    class TimestampedAccelerationData(
            val timestamp: Double,
            velocity: Double,
            power: Double,
            acceleration: Double
    ): DriveCharacterization.AccelerationDataPoint(velocity, power, acceleration) {
        fun toCSV() = "$timestamp,$velocity,$power,$acceleration"
    }

    val data = arrayListOf<TimestampedAccelerationData>()

    private var startTime = 0.0
    private var prevTime = 0.0
    private var prevVelocity = 0.0

    override fun entry(currentTime: Double) {
        data.clear()
        println("Starting linear acceleration test at power $power for $timeSeconds seconds")
        prevTime = 0.0
        prevVelocity = 0.0
        startTime = currentTime
        prevTime = startTime
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        if (currentTime - startTime > timeSeconds) {
            //We are done.
            drive.stop()
            return true
        }
        val currentVelocity = (Math.abs(drive.left.getVelocity().value) + Math.abs(drive.right.getVelocity().value)) / 4096.0 * Math.PI * 10.0

        if (prevTime == startTime) {
            prevTime = currentTime
            prevVelocity = currentVelocity
            println("Points populated")
            return false //First run, populate points
        }

        val acceleration = (currentVelocity - prevVelocity) / (currentTime - prevTime)

        if (acceleration < Util.kEpsilon) {
            //Ignore accelerations too small
            prevTime = currentTime
            prevVelocity = currentVelocity
            return false
        }

        data.add(TimestampedAccelerationData(
                currentTime,
                currentVelocity,
                power * 12.0,
                acceleration
        ))

        prevTime = currentTime
        prevVelocity = currentVelocity
        return false
    }

    override fun exit(currentTime: Double) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}