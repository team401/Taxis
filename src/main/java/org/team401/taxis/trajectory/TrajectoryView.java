//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.trajectory;

import org.team401.taxis.geometry.State;

public interface TrajectoryView<S extends State<S>> {
    public TrajectorySamplePoint<S> sample(final double interpolant);

    public double first_interpolant();

    public double last_interpolant();

    public Trajectory<S> trajectory();
}
