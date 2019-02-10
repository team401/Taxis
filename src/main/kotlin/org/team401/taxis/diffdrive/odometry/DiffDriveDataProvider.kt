package org.team401.taxis.diffdrive.odometry

import org.snakeskin.component.ISensoredGearbox
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 1/4/19
 */
class DiffDriveDataProvider(private val drivetrain: IPathFollowingDiffDrive<ISensoredGearbox>): DriveDataProvider {
    override fun getLeftPositionInches(): Double {
        return drivetrain.left.getPosition().toLinearDistance(drivetrain.wheelRadius).value
    }

    override fun getRightPositionInches(): Double {
        return drivetrain.right.getPosition().toLinearDistance(drivetrain.wheelRadius).value
    }

    override fun getLeftVelocityIps(): Double {
        return drivetrain.left.getVelocity().toLinearVelocity(drivetrain.wheelRadius).value
    }

    override fun getRightVelocityIps(): Double {
        return drivetrain.right.getVelocity().toLinearVelocity(drivetrain.wheelRadius).value
    }

    override fun getGyroHeading(): Rotation2d {
        return drivetrain.getHeading()
    }
}