package org.team401.taxis.diffdrive.control

import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.TimedState

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 * Reference implementation of the PathController interface, just does nothing
 */
class NoOpPathController: PathController {
    override fun update(dt: Double, dynamics: DifferentialDrive.DriveDynamics, currentState: Pose2d, model: DifferentialDrive, trajectory: TrajectoryIterator<TimedState<Pose2dWithCurvature>>, setpoint: TimedState<Pose2dWithCurvature>, error: Pose2d, reversed: Boolean): Output {
        return Output()
    }
}