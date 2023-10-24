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
    {0.5,125.0},
    {1.0,105.0},
    {1.5,73.0},
    {2.0,53.0},
    {2.5,41.0},
    {3.0,32.0},
    {3.5,26.5},
    {4.0,25.0}

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

   public double getDistance(double v) {
      if (v > distanceTableUpright[0][1]){
        return 0.0;
      }
      if (v < distanceTableUpright[distanceTableUpright.length-1][1]){
        return 10.0; 
      }
      for(int i = 0; i < distanceTableUpright.length - 1; i++){
        if (distanceTableUpright[i][1] <= v && v <= distanceTableUpright[i+1][1]){
          return (distanceTableUpright[i][0]+distanceTableUpright[i+1][0])/2.0;
        
        }
      }
      return 100;
   }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getVisionTargetXBoxRough();
    getVisionTargetYBoxRough();
    isUpRight();
    SmartDashboard.putNumber("***See Distance: ", getDistance(getVisionTargetYBoxRough()));
  }
}
