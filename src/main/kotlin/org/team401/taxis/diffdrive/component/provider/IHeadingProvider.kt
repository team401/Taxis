package org.team401.taxis.diffdrive.component.provider

import org.team401.taxis.geometry.Rotation2d

/**
 * Represents a device that can provide a heading.  Headings are represented as Rotation2d objects, and are guarenteed
 * to handle wrapping across multiple robot rotations properly.  This is different from the behavior of IYawProvider.
 */
interface IHeadingProvider {
    /**
     * Gets the current heading
     */
    fun getHeading(): Rotation2d

    /**
     * Sets (offsets) the current heading to the provided heading
     */
    fun setHeading(heading: Rotation2d)
}