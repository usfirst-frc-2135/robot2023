package frc.robot.team254.lib.geometry;

import frc.robot.team254.lib.util.Interpolable;
import frc.robot.team254.lib.util.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
