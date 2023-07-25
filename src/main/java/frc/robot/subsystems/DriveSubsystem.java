// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveMods;
  
  /**
   * Creates a new DriveSubsystem.
   * In a Swerve this subsystem represents chassis of the Swerve drive.
   * Here we organize the Swerve modules in a proper array object, and define methods that can command
   * individual swerve modules.
   * If a different subsystem needs to operate with the individual swerve module, it needs to go through
   * this subsystem, as it knows which module is responsible for which wheel.
   * If one wants to apply changes to the calculated power and angle for the individual swerve modules,
   * such as implementing a joystick deadzone in order to reduce excessive wheel movements, this
   * should be done here as well. Such decisions should be done by the chassis, not by the individual
   * swerve modules. The swerve modules essentially should simply do what they're told. The only optimization
   * that may be implemented on the swerve module level is angle rotaiton minimization, because it does not
   * change the result of the state change.
   */
  public DriveSubsystem() {
    
    swerveMods = new SwerveModule[] {
      new SwerveModule(0, SwerveModuleConstants.MOD0),  // front left
      new SwerveModule(1, SwerveModuleConstants.MOD1),  // front right
      new SwerveModule(2, SwerveModuleConstants.MOD2),  // rear left
      new SwerveModule(3, SwerveModuleConstants.MOD3)   // rear right
    };
  }

  public double telemetryAngleEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoder();
  }

  public double telemetryAngleEncoderSI(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoderSI();
  }

  public double telemetryDriveEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryDriveEncoder();
  }

  public void testDriveMotorEncoderPhase(int modnumber){
    swerveMods[modnumber].testDriveMotorApplyPower(0.3);
  }

  public void stopDriveMotor(int modnumber){
    swerveMods[modnumber].testDriveMotorApplyPower(0);
  }

  public void testAngleMotorEncoderPhase(int modnumber) {
    swerveMods[modnumber].testAngleMotorApplyPower(0.3);
  }

  public void stopAngleMotor(int modnumber) {
    swerveMods[modnumber].testAngleMotorApplyPower(0);
  }

  /**
   * X and Y velocity values need to be submitted from a field point of view, where the 0,0 coordinates are in the 
   * left lower corner of the field.
   * @param xVelocity_m_per_s
   * @param yVelocity_m_per_s
   * @param omega_rad_per_s
   */
  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    SwerveModuleState[] swerveModuleStates = Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity_m_per_s,
            yVelocity_m_per_s,
            omega_rad_per_s,
            Rotation2d.fromDegrees(RobotContainer.imuSubsystem.getYaw())
            )
    );
  
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.MAX_SPEED);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]); 
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
