package org.team401.taxis.diffdrive.component

import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.taxis.diffdrive.component.provider.IHeadingProvider
import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 9/15/18
 *
 * Represnts a differential drivetrain that has odometry and path following models.
 */
interface IModeledDifferentialDrivetrain: IDifferentialDrivetrain {
    /**
     * The heading source for this drivetrain
     */
    val headingSource: IHeadingProvider

    /**
     * The physics and kinematics model for this drivetrain
     */
    val model: DifferentialDrivetrainModel

    /**
     * The DriveState implementation used to manage the pose of this drivetrain
     */
    val driveState: DifferentialDriveState

    /**
     * Gets the current heading of the drivetrain
     */
    fun getHeading(): Rotation2d = headingSource.getHeading()

    /**
     * Sets the heading of the drivetrain to the given value
     */
    fun setHeading(heading: Rotation2d) = headingSource.setHeading(heading)

    /**
     * Sets the pose of the drivetrain to the provided value.
     * Use this function instead of manipulating the DriveState directly to prevent issues with the heading
     * changing without the DriveState knowing about it.
     *
     * @param pose The new pose to set
     * @param time The time to mark the current pose as valid for
     */
    fun setPose(pose: Pose2d, time: TimeMeasureSeconds) {
        setHeading(pose.rotation)
        driveState.reset(time, pose)
    }

    /**
     * Gets the pose of the drive at the specified timestamp
     */
    fun getPose(time: TimeMeasureSeconds): Pose2d {
        return driveState.getFieldToVehicle(time)
    }
}