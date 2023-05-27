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
      new SwerveModule<Constants.Swerve.Mod0>(0, new Constants.Swerve.Mod0()),
      new SwerveModule<Constants.Swerve.Mod1>(1, new Constants.Swerve.Mod1()),
      new SwerveModule<Constants.Swerve.Mod2>(2, new Constants.Swerve.Mod2()),
      new SwerveModule<Constants.Swerve.Mod3>(3, new Constants.Swerve.Mod3())
};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
