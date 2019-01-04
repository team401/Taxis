package org.team401.taxis.diffdrive.odometry

import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.LinearVelocityUnit
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive
import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 1/4/19
 */
class DiffDriveDataProvider(private val drivetrain: PathFollowingDiffDrive): DriveDataProvider {
    override fun getLeftPositionInches(): Double {
        return drivetrain.left.getPosition().toLinearDistance(drivetrain.wheelRadius).toUnit(LinearDistanceUnit.Standard.INCHES).value
    }

    override fun getRightPositionInches(): Double {
        return drivetrain.right.getPosition().toLinearDistance(drivetrain.wheelRadius).toUnit(LinearDistanceUnit.Standard.INCHES).value
    }

    override fun getLeftVelocityIps(): Double {
        return drivetrain.left.getVelocity().toLinearVelocity(drivetrain.wheelRadius).toUnit(LinearVelocityUnit.Standard.INCHES_PER_SECOND).value
    }

    override fun getRightVelocityIps(): Double {
        return drivetrain.right.getVelocity().toLinearVelocity(drivetrain.wheelRadius).toUnit(LinearVelocityUnit.Standard.INCHES_PER_SECOND).value
    }

    override fun getGyroHeading(): Rotation2d {
        return drivetrain.getHeading()
    }
}