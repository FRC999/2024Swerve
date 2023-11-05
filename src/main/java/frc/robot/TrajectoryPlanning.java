package frc.robot;

import java.util.List;

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

}
