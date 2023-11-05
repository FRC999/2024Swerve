// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NetworkTablesSubsystem extends SubsystemBase {
  /** Creates a new NetworkTablesSystem. */
  private NetworkTableInstance ntInst;
  private double[][] distanceTableUpright ={
    {0.5,130.0},
    {1.0,100.0},
    {1.5,66.0},
    {2.0,50.0},
    {2.5,40.0},
    {3.0,35.0},
    {3.5,31.0}

  };

  private double[][] distanceTableSide ={
    {0.5,85.0},
    {1.0,62.0},
    {1.5,43.0},
    {2.0,27.0},
    {2.5,23.0}

  };

  public NetworkTablesSubsystem() {
    ntInst = NetworkTableInstance.getDefault();
    System.out.println(); //TODO: print the array parameters
  }
  public boolean seeCone() {
    double tv = ntInst.getTable("limelight").getEntry("tv").getDouble(0);
    SmartDashboard.putBoolean("***See Cone Result: ", tv == 1);
    return tv == 1;
  }

  public double getVisionTargetX() {
    double tx = ntInst.getTable("limelight").getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("***See TX Result: ", -tx);
    return -tx;
  }

  public double getVisionTargetXBoxRough() {
    double thor = ntInst.getTable("limelight").getEntry("thor").getDouble(0);
    SmartDashboard.putNumber("***See Thor Result: ", thor);
    return thor;
  }
  public double getVisionTargetYBoxRough() {
    double tvert = ntInst.getTable("limelight").getEntry("tvert").getDouble(0);
    SmartDashboard.putNumber("***See Tvert Result: ", tvert);
    return tvert;
  }

  public boolean isUpRight() {
    boolean upright = getVisionTargetYBoxRough()/getVisionTargetXBoxRough()>1.1;
    SmartDashboard.putBoolean("***IsUpRight", upright);
    return upright;
  }

  public double getDistance(double v, double h) {

    System.out.println("v: " + v + " h: " + h);

    if (v == 0 || h == 0) { // Do not see the cone
      return Double.NaN;
    }
    double[][] distanceTable;
    if (v / h >= 1.3) { // Cone is vertical

      distanceTable = distanceTableUpright;
      System.out.println("Vertical");
    } else {
      distanceTable = distanceTableSide;
      System.out.println("Horizontal");
    }
    if (v > distanceTable[0][1]) {
      return 0.0;
    }
    if (v < distanceTable[distanceTable.length - 1][1]) {
      return 10.0;
    }
    for (int i = 0; i < distanceTable.length - 1; i++) {
      if (distanceTable[i][1] >= v && v >= distanceTable[i + 1][1]) {
        return (distanceTable[i][0] + (distanceTable[i + 1][0]-distanceTable[i][0])
                *(v-distanceTable[i][1])/
                (distanceTable[i+1][1]-distanceTable[i][1]));

      }
    }
    return 100;
  }

  public double getDistanceToCone() {
    return getDistance(getVisionTargetYBoxRough(), getVisionTargetXBoxRough());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getVisionTargetXBoxRough();
    getVisionTargetYBoxRough();
    isUpRight();
    SmartDashboard.putNumber("***See Distance: ", getDistanceToCone());
  }
}
