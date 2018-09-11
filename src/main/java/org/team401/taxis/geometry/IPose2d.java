//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public Pose2d getPose();

    public S transformBy(Pose2d transform);

    public S mirror();
}
