package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.team401.taxis.diffdrive.autotune.TuneTrackScrubFactor

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 */
class TuningAutoTuneTrackScrubFactor(val drivetrain: TankDrivetrain,
                                     val turns: Int = 10,
                                     val power: Double = 1.0,
                                     val direction: Boolean = true): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return SequentialSteps(
                TuneTrackScrubFactor(drivetrain, turns, power, direction)
        )
    }
}