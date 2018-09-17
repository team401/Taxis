package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasure
import org.team401.taxis.diffdrive.autotune.TuneWheelRadius

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 * Full auto mode to tune the wheel radius
 */
class TuningAutoTuneWheelRadius(val drive: TankDrivetrain, val distance: LinearDistanceMeasure): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return SequentialSteps(TuneWheelRadius(drive, distance))
    }

}