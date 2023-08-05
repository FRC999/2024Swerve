package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.SwerveModuleConstants;
import frc.robot.PassThroughSystems.Motor.BaseMotorPassthrough;


public class SwerveModule extends SubsystemBase {

    private BaseMotorPassthrough driveMotor;
    private BaseMotorPassthrough angleMotor;

    private int moduleNumber;
    private double angleOffset;

    private double lastAngle;

    private Rotation2d currentAngle = new Rotation2d();

    // This is not used for the manual teleop driving
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,
            Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    /**
     * This subsystem instantiates and manages a single swerve module.
     * Since you have multiple swerve modules, this class is probably instantiated in a loop.
     * It provides logic for the swerve operation including state changes.
     * It is defined as a Subsystem in case you want to have a periodic code in it.
     * This class should instantiate all relevant motors. Since relevant encoders are only
     * used for driving, they're exposed via corresponding motor objects, rather than separate objects.
     * However, since IMU may be used for other functions besides driving, IMU is exposed via
     * a separate subsystem, instantiated in RobotContainer via public static final object.
     * Hence, all IMU-related calls need to use that object.
     * 
     * @param moduleNumber - identifies module numerically (e.g. 0,1,2,3) - primarily used for logging
     * @param moduleConstants - supplies module configuration parameters via enum defined in Constants
     */
    public SwerveModule(int moduleNumber, Constants.Swerve.SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.getAngleOffset();

        driveMotor = new BaseMotorPassthrough(moduleConstants.getDriveMotorControllerType(), moduleConstants.getDriveMotorID());

        angleMotor = new BaseMotorPassthrough(moduleConstants.getAngleMotorControllerType(), moduleConstants.getAngleMotorID());

        driveMotor.configureDriveMotor(moduleConstants);

        angleMotor.configureAngleMotor(moduleConstants);

        lastAngle = getState().angle.getDegrees();

    }

    public SwerveModuleState getState() {
        double velocity = driveMotor.getDriveEncoderVelocitySI();
        Rotation2d angle = Rotation2d.fromDegrees(angleMotor.getAngleEncoderPositionSI());
        return new SwerveModuleState(velocity, angle);
    }

    public double telemetryAngleEncoder(){
        return angleMotor.getAngleEncoderPosition();
    }

    public double telemetryAngleEncoderSI(){
        return angleMotor.getAngleEncoderPositionSI();
    }

    public double telemetryDriveEncoder(){
        return driveMotor.getDriveEncoderPosition();
    }

    public void testDriveMotorApplyPower(double power) {
        driveMotor.testMotorApplyPower(power);
    }

    public void testAngleMotorApplyPower(double power) {
        angleMotor.testMotorApplyPower(power);
    }

    public int getModuleNumber(){
        return moduleNumber;
    }

    /**
     * Set angle motor in swerve module to a specific angle and set drive motor to a specific power.
     * These settings are robot centric and need to be converted from the field centric values by the chassis
     * code located in DriveSubsystem.
     * This method will apply power to the motors by calling appropriate methods in both the drive and angle motor objects.
     * SwerveModule should not be making decisions to change the drive speed or direction (besides optimization), based on
     * other factors such as joystick deadzones, because it cannot inform other instances of SwerveModule of the change.
     * Such decisions should be done on the level of the drive chassis (DriveSubsystem).
     * 
     * @param desiredState - The wheel angle and the speed we want the swerve module to go to.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Minimizes angle movement of angle motor by limiting movement to 90 degrees and 
        // reversing power to negative value if necessary
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // Use this flag for chassis testing, if you want to see the angle and power numbers provided by the Swerve calculations
        // instead of actually moving the robot
        if (Constants.Swerve.TalonSRXConfiguration.testSwervePrintOnly) {
            System.out.print(" SM: "+moduleNumber);
            System.out.print(" P: "+desiredState.speedMetersPerSecond / Swerve.MAX_SPEED);
            System.out.println(" A: "+desiredState.angle.getDegrees());
        } else {

            // This is the code that makes the robot move by applying the power to the motors
            driveMotor.applyPower(desiredState.speedMetersPerSecond / Swerve.MAX_SPEED);
            angleMotor.setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not give degrees
        }
    }

    public SwerveModulePosition getPosition() {
        double position = driveMotor.getDriveEncoderPosition(); 
        Rotation2d angle = currentAngle;
        return new SwerveModulePosition(position, angle);
    }

    @Override
    public void periodic() {

        // While we update the current angle of the angle motor for telemetry,
        // It's not used in the teleop driving, as we use real-time update via getState call.
        currentAngle = Rotation2d.fromDegrees(angleMotor.getAngleEncoderPositionSI());

        //integratedAngleEncoder.setPosition(cancoder.getAbsolutePosition() - angleOffset);
    }
}
