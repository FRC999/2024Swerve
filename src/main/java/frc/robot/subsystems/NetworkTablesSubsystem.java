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
  public NetworkTablesSubsystem() {
    ntInst = NetworkTableInstance.getDefault();
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

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
