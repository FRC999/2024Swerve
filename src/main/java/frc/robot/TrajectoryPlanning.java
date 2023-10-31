package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryPlanning {
    public static double[] trajectoryCoordinates(double distance, double imuAngle){
        double[] coordinates = new double[2];
        coordinates[0] = distance*Math.cos(imuAngle);
        coordinates[1] = distance*Math.sin(imuAngle);
        return coordinates;
    }

    public static PathPlannerTrajectory trajectoryToGameElement(double[] endCoordinates) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
    new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
    new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
);
    }
}
