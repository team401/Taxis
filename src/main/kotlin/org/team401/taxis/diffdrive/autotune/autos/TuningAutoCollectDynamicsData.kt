package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.team401.taxis.diffdrive.autotune.CollectDynamicsData

/**
 * @author Cameron Earle
 * @version 12/4/2018
 *
 */
class TuningAutoCollectDynamicsData(val drivetrain: TankDrivetrain): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return SequentialSteps(CollectDynamicsData(drivetrain))
    }
}