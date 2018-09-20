package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.LambdaStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.hardware.Hardware
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.diffdrive.autotune.CollectLinearAccelerationData
import java.io.File

/**
 * @author Cameron Earle
 * @version 9/19/2018
 *
 */
class AutoTestCollectAccel(val drive: TankDrivetrain,
                           val power: Double,
                           val runtime: TimeMeasure): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        val collect = CollectLinearAccelerationData(drive, power, runtime)
        return SequentialSteps(
                collect,
                LambdaStep {
                    //Log data
                    val f = File("/home/lvuser/LinearAcceleration-${Hardware.getAbsoluteTime()}.csv")
                    val printer = f.printWriter()
                    printer.println("Timestamp (s),Velocity (rad/s),Voltage,Acceleration (rad/s/s)")
                    collect.data.forEach {
                        printer.println(it.toCSV())
                    }
                    printer.flush()
                    printer.close()
                }
        )
    }
}