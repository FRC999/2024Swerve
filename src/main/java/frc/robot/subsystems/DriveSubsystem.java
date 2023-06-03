// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve.*;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public SwerveModule[] swerveMods;
  
  public DriveSubsystem() {
    
    swerveMods = new SwerveModule[] {
      new SwerveModule(0, SwerveModuleConstants.MOD0),
      new SwerveModule(1, SwerveModuleConstants.MOD1),
      new SwerveModule(2, SwerveModuleConstants.MOD2),
      new SwerveModule(3, SwerveModuleConstants.MOD3)
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
