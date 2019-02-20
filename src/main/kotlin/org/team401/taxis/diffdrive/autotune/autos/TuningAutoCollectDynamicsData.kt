package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.ICTREGearbox
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.component.ISmartGearbox
import org.team401.taxis.diffdrive.autotune.CollectDynamicsData

/**
 * @author Cameron Earle
 * @version 12/4/2018
 *
 */
class TuningAutoCollectDynamicsData(val drivetrain: IDifferentialDrivetrain<ISmartGearbox<*>>): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return SequentialSteps(CollectDynamicsData(drivetrain))
    }
}