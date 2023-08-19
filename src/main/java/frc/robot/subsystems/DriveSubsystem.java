// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public void stopRobot() {
    drive(0,0,0, true);
  }

  /**
   * This method calculates the desired state for each swerve module depending on joystic inputs.
   * In other words, it translates the joystic inputs into the desired state for each of the individual swerve modules.
   * X and Y velocity values need to be submitted from a field point of view, where the 0,0 coordinates are in the 
   * left lower corner of the field.
   * @param xVelocity_m_per_s
   * @param yVelocity_m_per_s
   * @param omega_rad_per_s
   * @param fieldcentric - if true, use field-centric swerve; otherwise - robot-centric
   */
  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric) {
    SwerveModuleState[] swerveModuleStates;

    if (fieldcentric) { // field-centric swerve
      swerveModuleStates = Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s,
              Rotation2d.fromDegrees(RobotContainer.imuSubsystem.getYaw())));
    } else { // robot-centric swerve; does not use IMU
      swerveModuleStates = Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s));
    }
  
    // Because the resulting power is a vector sum of the robot direction and rotation, it's possible that
    // the resulting vector would exceed absolute scalar value of 1. And we need to keep the power in the -1..+1 range.
    // Hence if any of the vectors have a scalar value greater than 1, we need to divide all of them by the largest scalar value
    // of a vector we have. That will preserve the direction of the robot even it is concurrently combioned with the
    // holonomic rotation.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.MAX_SPEED);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]); 
    }

  }

  /**
   * This method sets desired states for the individual swerve modules, specifying angle for the angle motor and power for the drive motor.
   * However, instead of actually driving, it will simply print what did we tell individual swerve modules. So, it should be used for testing.
   * @param swerveModuleStates - array of the desired swerve module states
   */
  public void setDesiredStatesCalibration(SwerveModuleState[] swerveModuleStates) {
  
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.MAX_SPEED);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredStateCalibration(swerveModuleStates[mod.getModuleNumber()]); 
    }

  }

  /**
   * This method sets desired states for the individual swerve modules, specifying angle for the angle motor and power for the drive motor.
   * In other words, this method tells each individual swerve module what to do next, and is directly involved in driving.
   * @param swerveModuleStates - array of the desired swerve module states
   */
  public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
  
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

  /**
   * Print values that we will use to update odometry.
   */
  public void odometryTelemetry() {
    System.out.println("Odometry: "+swerveOdometry.getPoseMeters());
  }

  public void odometryCommandTelemetry(Rotation2d r, SwerveModulePosition[] s) {
    System.out.println("Odometry Command Rotation: "+r);
    
    for(int i = 0; i < 4; i++)  {
      System.out.println("Odometry Command Module: "+ i + " " + s[i]);
    }
  
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


    
    //TODO: We may want to update the robot odometry based the cameras and AprilTags

    /**
     * This will produce the IMU and swerve positions we send to odometry along with the same information obtained from telemetry after update.
     * It may help troubleshooting potential odometry update issues (e.g. units of measure issues etc)
     */
    if (SwerveTelemetry.odometryTelemetryPrint) {
      Rotation2d r = RobotContainer.imuSubsystem.getYawRotation2d();
      SwerveModulePosition[] s = getPositions();
      odometryCommandTelemetry(r, s);
      swerveOdometry.update(r, s);
      odometryTelemetry();
    } else {
      swerveOdometry.update(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());
    }
  }
}
