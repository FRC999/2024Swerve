// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveToTarget extends CommandBase {

  private double distance; //distance to target
  private double angle; //angle to target
  private double chassisAngle;
  private final double stoppingDistance = 0.1;
  private final double kP = 0.5;
	private final double kI = 0;
	private final double kD = 0;
  private double kMaxSpeed = 1.5;
	private double kMaxAccel = 3.0;
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
  private ProfiledPIDController profiledPID = new ProfiledPIDController(kP, kI, kD, constraints);
  private Pose2d initialPose2d;
  private Pose2d finalPose2d;
  private Translation2d finalTranslation2d;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(double dst, double ang) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
    
    distance = dst;
    angle = Math.toRadians(ang);
    chassisAngle = RobotContainer.imuSubsystem.getYawRotation2d().getRadians();
    //reset odometry to 0,0 coordinates
    RobotContainer.driveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(angle)));
    
    initialPose2d = new Pose2d(0.0,0.0,new Rotation2d(chassisAngle));
    finalPose2d = new Pose2d(distance * Math.cos(chassisAngle + angle),distance * Math.sin(chassisAngle + angle),new Rotation2d(chassisAngle + angle) );
    finalTranslation2d = finalPose2d.getTranslation();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profiledPID.reset(-distance);
    profiledPID.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    RobotContainer.driveSubsystem.updateTrajectoryOdometry();
    Pose2d currentPose2d = RobotContainer.driveSubsystem.getPose();
    double pidError = -finalTranslation2d.
          getDistance(currentPose2d.getTranslation());
    if (initialPose2d.getTranslation().
          getDistance(currentPose2d.getTranslation())>distance){
      pidError = -pidError;
    }
    double power = profiledPID.calculate(pidError);
  
    System.out.println("p:"+power);

    RobotContainer.driveSubsystem.drive(power*Math.cos(chassisAngle + angle), 
        power*Math.sin(chassisAngle + angle), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
