package org.team401.taxis.diffdrive.component.impl

import org.snakeskin.component.provider.IYawProvider
import org.team401.taxis.diffdrive.component.provider.IHeadingProvider
import org.team401.taxis.geometry.Rotation2d

/**
 * Implements a heading source from a yaw provider, such as an IMU.
 */
class YawHeadingSource(private val yawProvider: IYawProvider): IHeadingProvider {
    private var offset = Rotation2d.identity()

    override fun getHeading(): Rotation2d {
        return Rotation2d.fromDegrees(yawProvider.getYaw().value).rotateBy(offset)
    }

    override fun setHeading(heading: Rotation2d) {
        offset = heading.rotateBy(Rotation2d.fromDegrees(yawProvider.getYaw().value).inverse())
    }
}