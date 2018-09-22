package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.DelayStep
import org.snakeskin.auto.steps.LambdaStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.Seconds
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.diffdrive.autotune.CollectLinearAccelerationData
import org.team401.taxis.diffdrive.autotune.CollectLinearStictionData
import org.team401.taxis.physics.DriveCharacterization

/**
 * @author Cameron Earle
 * @version 9/22/2018
 *
 */
class TuningAutoTuneCharacterization(val drivetrain: TankDrivetrain,
                                     val stictionPower: Double,
                                     val stictionRampRate: Double,
                                     val accelPower: Double,
                                     val accelRuntime: TimeMeasure): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        val collectStiction = CollectLinearStictionData(drivetrain, stictionPower, stictionRampRate)
        val collectAccel = CollectLinearAccelerationData(drivetrain, accelPower, accelRuntime)
        return SequentialSteps(
                collectStiction,
                DelayStep(20.Seconds),
                collectAccel,
                LambdaStep {
                    val characterization = DriveCharacterization.characterizeDrive(collectStiction.data, collectAccel.data as ArrayList<DriveCharacterization.AccelerationDataPoint>)
                    println("kS: ${characterization.ks}")
                    println("kV: ${characterization.kv}")
                    println("kA: ${characterization.ka}")
                }
        )
    }
}