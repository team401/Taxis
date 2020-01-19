//File originally from FRC Team 254's 2019 Robot code

package org.team401.taxis.geometry;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
