package org.team401.taxis.geometry;

import org.team401.taxis.util.CSVWritable;
import org.team401.taxis.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
