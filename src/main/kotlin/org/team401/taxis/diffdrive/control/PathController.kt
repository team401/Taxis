package org.team401.taxis.diffdrive.control

import org.team401.taxis.diffdrive.control.Output
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.TimedState

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 * Common interface for all path controllers.  Path controllers are used by a path follower to define
 * the feedback loop that the path uses.  A controller will be provided with all the data necessary
 * to run feedback.
 */

interface PathController {
    /**
     * Update function defining the feedback.  Takes in a series of inputs about the system,
     * and returns a single output reflecting the next command for the system.
     *
     * The following parameters will be provided:
     *
     * @param dt The delta time, in seconds
     * @param model The control of the differential drivetrain
     * @param trajectory The current trajectory
     * @param setpoint The current setpoint of the trajectory
     * @param error The current error from the setpoint
     * @param reversed True if the current trajectory is reversed
     *
     * The following must be returned:
     *
     * @return The output signal to send to the drive
     */
    fun update(dt: Double,
               dynamics: DifferentialDrive.DriveDynamics,
               currentState: Pose2d,
               model: DifferentialDrive,
               trajectory: TrajectoryIterator<TimedState<Pose2dWithCurvature>>,
               setpoint: TimedState<Pose2dWithCurvature>,
               error: Pose2d,
               reversed: Boolean): Output

    /**
     * Reset function, provided as a convenience to reset any local variables when starting a new path.
     * Will be called by the PathFollower when a new path is about to start.
     *
     * Default implementation is no-op
     */
    fun reset() {}
}