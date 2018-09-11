//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.trajectory;

import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
