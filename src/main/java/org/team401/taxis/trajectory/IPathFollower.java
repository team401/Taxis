package org.team401.taxis.trajectory;

import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
