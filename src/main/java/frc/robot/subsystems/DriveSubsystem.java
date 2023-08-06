// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveMods;

  public SwerveDriveOdometry swerveOdometry;
  public SwerveDrivePoseEstimator swervePoseEstimator;
  
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

    // When the robot is turned on, both the IMU and drive encoders will be set to 0.
    // So, the initial odometry X,Y,Angle will be set to 0,0,0
    // This may need to be updated later either by th auto-driviing routines, or by camera inputs based on the AprilTags
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());

    // This object will help tracking Swerve pose changes based on the odometry
    // So it will likely be used only for telemetry
    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), new Pose2d());
  
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

  // Used only for motor testing; run motor forward, 0.3 power
  public void testDriveMotorEncoderPhase(int modnumber){
    swerveMods[modnumber].testDriveMotorApplyPower(0.3);
  }

  // Used only for motor testing; run motor forward, 0.3 power
  public void testAngleMotorEncoderPhase(int modnumber) {
    swerveMods[modnumber].testAngleMotorApplyPower(0.3);
  }

  public void stopDriveMotor(int modnumber){
    swerveMods[modnumber].DriveMotorApplyPower(0);
  }

  public void stopAngleMotor(int modnumber) {
    swerveMods[modnumber].AngleMotorApplyPower(0);
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

  /**
   * Creates an array of the swerve module positions (one array element per swerve module)
   * It is only used in odometry calculations, meaning, is only used for automated/trajectory driving
   * and not for teleop/manual driving
   * 
   * @return SwerveModulePosition[] - array of the SwerveModulePosition objects (WPILIB)
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++) {
        positions[i] = swerveMods[i].getPosition();
    }
    return positions;
  }

  // Set odometry to a specified field-centric Pose2d
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), pose);
  }

  // Field Centric Pose
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetPoseEstimator(Pose2d pose) {
    swervePoseEstimator.resetPosition(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), pose);
  }

  public Pose2d getPoseEstimate() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveOdometry.update(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());
    //TODO: We may want to update the robot odometry based the cameras and AprilTags

  }
}
