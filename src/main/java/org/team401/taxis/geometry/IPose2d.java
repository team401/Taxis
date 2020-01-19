//File originally from FRC Team 254's 2019 Robot code

package org.team401.taxis.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    Pose2d getPose();

    S transformBy(Pose2d transform);

    S mirror();
}
