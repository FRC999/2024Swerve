// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Swerve {
    
	// TRAJECTORY PARAMETERS 3039
	/* Drive Feedforward */
	public static final double DRIVE_KS = 0.11937 / 12;
	public static final double DRIVE_KV = 2.6335 / 12;
	public static final double DRIVE_KA = 0.46034 / 12;

    //drive train properties
    //in meters
    public static final double TRACK_WIDTH = 0.390; //left to right
		public static final double WHEEL_BASE = 0.537;  //front to back
		public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

      /* Ramp Rates and Current Limits */
		public static final double DRIVE_CLOSED_LOOP_RAMP = 0;
		public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
		public static final int ANGLE_MOTOR_SMART_CURRENT = 25;
		public static final double ANGLE_MOTOR_SECONDARY_LIMIT = 40;
		public static final int DRIVE_MOTOR_SMART_CURRENT = 40;
		public static final double DRIVE_MOTOR_SECONDARY_LIMIT = 60;

    /* Angle Motor PID */
		public static final double ANGLE_MOTOR_KP = 0.75;
		public static final double ANGLE_MOTOR_KI = 0.005;
		public static final double ANGLE_MOTOR_KD = 0.01;

    public static final double ANGLE_MOTOR_MIN_OUTPUT = -1;
		public static final double ANGLE_MOTOR_MAX_OUTPUT = 1;

    public static final double ANGLE_MOTOR_PID_TIMEOUT = 30; //milliseconds

    public static final double ANGLE_MOTOR_VELOCITY_CONVERSION = (360.0 * 10.0) / 4096.0; //conversion factor from tick/100ms to degree/s

    public static final double MAX_ANGULAR_VELOCITY = 11.5;
		public static final double MAX_SPEED = 3.0;

	public static enum BaseMotorControllerTypes{
		TALON_SRX,
		SPARKMAX;
	}

	/**
	 * Swerve Module Constants including
	 * driveMotorID - CAN ID of the drive motors
	 * angleMotorID - CAN ID of the rotation motors
	 * angleOffset - Angle deviation of the absolute encoder when the 
	 * respective wheel is pointing forward based on the absolute encoder value
	 */
	public static enum SwerveModuleConstants{
		MOD0(
			BaseMotorControllerTypes.TALON_SRX, // Drive motor type
			BaseMotorControllerTypes.TALON_SRX, // Angle motor type
			7, 									// driveMotorID
			8, 									// angleMotorID
			(3268.0 * 360.0) / 4096.0,  		// angleOffset
			false,								// Inversion for drive motor
			false								// Inversion for angle motor
		),
		MOD1(
			BaseMotorControllerTypes.TALON_SRX, // Drive motor type
			BaseMotorControllerTypes.TALON_SRX, // Angle motor type
			5, 									// driveMotorID
			6, 									// angleMotorID
			(433.0 * 360.0) / 4096.0,   		// angleOffset
			false,								// Inversion for drive motor
			false								// Inversion for angle motor
		),
		MOD2(
			BaseMotorControllerTypes.TALON_SRX, // Drive motor type
			BaseMotorControllerTypes.TALON_SRX, // Angle motor type
			1, 									// driveMotorID
			2, 									// angleMotorID
			(3489.0 * 360.0) / 4096.0, 			// angleOffset
			false,								// Inversion for drive motor
			false 								// Inversion for angle motor
		),
		MOD3(
			BaseMotorControllerTypes.TALON_SRX, // Drive motor type
			BaseMotorControllerTypes.TALON_SRX, // Angle motor type
			3, 									// driveMotorID
			4, 									// angleMotorID
			(3307.0 * 360.0) / 4096.0, 			// angleOffset
			false,								// Inversion for drive motor
			false 								// Inversion for angle motor
		);

		private BaseMotorControllerTypes driveBaseMotorControllerType;
		private BaseMotorControllerTypes angleBaseMotorControllerType;
		private int driveMotorID;
		private int angleMotorID;
		private double angleOffset;
		private boolean driveMotorInverted;
		private boolean angleMotorInverted;

		SwerveModuleConstants(BaseMotorControllerTypes dm, BaseMotorControllerTypes am, int d, int a, double o, boolean di, boolean ai){
			this.driveBaseMotorControllerType = dm;
			this.angleBaseMotorControllerType = am;
			this.driveMotorID = d;
			this.angleMotorID = a;
			this.angleOffset = o;
			this.driveMotorInverted = di;
			this.angleMotorInverted = ai;
		}

		public BaseMotorControllerTypes getDriveMotorControllerType() {
			return driveBaseMotorControllerType;
		}

		public BaseMotorControllerTypes getAngleMotorControllerType() {
			return angleBaseMotorControllerType;
		}

		public int getDriveMotorID() {
			return driveMotorID;
		}

		public int getAngleMotorID() {
			return angleMotorID;
		}

		public double getAngleOffset() {
			return angleOffset;
		}

		public boolean isDriveMotorInverted() {
			return driveMotorInverted;
		}

		public boolean isAngleMotorInverted() {
			return angleMotorInverted;
		}
	}
    /* Module Constants */
		// BEVEL GEARS FACE RADIO FOR OFFSETS
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 7;
			public static final int angleMotorID = 8;
			//public static final int canCoderID = 8;
			public static final double angleOffset = (3268.0 * 360.0) / 4096.0;
			//public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
				//	angleMotorID, angleOffset);

      public int getDriveMotorID() { return driveMotorID; }
      public int getAngleMotorID() { return angleMotorID; }
      public double getAngleOffset() { return angleOffset; }


		}

		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 5;
			public static final int angleMotorID = 6;
			//public static final int canCoderID = 1;
			public static final double angleOffset = (433.0 * 360.0) / 4096.0;
			//public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
				//	angleMotorID, angleOffset);

      public int getDriveMotorID() { return driveMotorID; }
      public int getAngleMotorID() { return angleMotorID; }
      public double getAngleOffset() { return angleOffset; }

		}

		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 1;
			public static final int angleMotorID = 2;
			//public static final int canCoderID = 2;
			public static final double angleOffset = (3489.0 * 360.0) / 4096.0;
			//public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
				//	angleMotorID, angleOffset);

      public int getDriveMotorID() { return driveMotorID; }
      public int getAngleMotorID() { return angleMotorID; }
      public double getAngleOffset() { return angleOffset; }
		}

		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 3;
			public static final int angleMotorID = 4;
			//public static final int canCoderID = 3;
			public static final double angleOffset = (3307.0 * 360.0) / 4096.0;
			//public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
				//	angleMotorID, angleOffset);

      public int getDriveMotorID() { return driveMotorID; }
      public int getAngleMotorID() { return angleMotorID; }
      public double getAngleOffset() { return angleOffset; }
		}

  }

  public static final class IMUConstants {
	
	public static enum IMUTypes{
		Pigeon2,
		NavX;
	}

	public static final IMUTypes imuType = IMUTypes.Pigeon2;
	public static final class Pigeon2Constants{
		public static final int pigeonIMUId = 15; //TODO: Make sure Pigeon ID is correct, this value is a placeholder
	}
  }

}
