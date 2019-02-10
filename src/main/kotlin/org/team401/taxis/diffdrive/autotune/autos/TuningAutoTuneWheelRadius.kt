package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.taxis.diffdrive.autotune.TuneWheelRadius

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 * Full auto mode to tune the wheel radius
 */
class TuningAutoTuneWheelRadius(val drive: IDifferentialDrivetrain<ISensoredGearbox>, val distance: LinearDistanceMeasureInches): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return SequentialSteps(TuneWheelRadius(drive, distance))
    }

}