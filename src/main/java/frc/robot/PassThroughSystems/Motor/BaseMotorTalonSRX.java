package frc.robot.PassThroughSystems.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.PIDConstants.SRXAngle;
import frc.robot.Constants.Swerve.SwerveModuleConstants;
import frc.robot.Constants.Swerve.TalonSRXConfiguration;

/*
 * This is a specific base motor implementation for the motors connected to TalonSRX
 */
public class BaseMotorTalonSRX implements BaseMotorInterface {
    private WPI_TalonSRX motorTalonSRX;

    public BaseMotorTalonSRX(int CANID) {

        System.out.println("**** Activating TalonSRX CANID:" + CANID);

        motorTalonSRX = new WPI_TalonSRX(CANID);
    }

    public void configureDriveMotor(Constants.Swerve.SwerveModuleConstants c) {

        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isDriveMotorInverted());

        // Encoder configuration
        motorTalonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
                Constants.Swerve.TalonSRXConfiguration.kPIDLoopIdx,
                Constants.Swerve.TalonSRXConfiguration.configureTimeoutMs);

        motorTalonSRX.setSensorPhase(c.getDriveMotorSensorPhase());

        motorBrakeMode();

    }

    public void configureAngleMotor(SwerveModuleConstants c) {

        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isAngleMotorInverted());

        // Encoder configuration
        motorTalonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute,
                Constants.Swerve.TalonSRXConfiguration.kPIDLoopIdx,
                Constants.Swerve.TalonSRXConfiguration.configureTimeoutMs);

        motorTalonSRX.setSensorPhase(c.getAngleMotorSensorPhase());

        initQuadrature();

        configureMotionMagicAngle(c);

        setEncoderforWheelCalibration(c);

        motorBrakeMode();

        //setAngleMotorChassisAngle(0); //Initialization turn wheels to 0 degrees
        
    }

    public double getDriveEncoderPosition() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getAngleEncoderPosition() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getDriveEncoderVelocity() {
        return motorTalonSRX.getSelectedSensorVelocity();
    }

    public double getAngleEncoderVelocity() {
        return motorTalonSRX.getSelectedSensorVelocity();
    }

    public double getDriveEncoderPositionSI() {
        return motorTalonSRX.getSelectedSensorPosition()*Constants.Swerve.TalonSRXConfiguration.metersPerTick;
    }

    public double getAngleEncoderPositionSI() {
        return motorTalonSRX.getSelectedSensorPosition()*Constants.Swerve.TalonSRXConfiguration.degreePerTick;
    }

    public double getDriveEncoderVelocitySI() {
        return motorTalonSRX.getSelectedSensorVelocity()*10.0*Constants.Swerve.TalonSRXConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
        return motorTalonSRX.getSelectedSensorVelocity()*10.0*Constants.Swerve.TalonSRXConfiguration.degreePerTick;
    }

    private int getDriveAbsEncoder() {
        return (int) motorTalonSRX.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    }

    public void setAngleMotorChassisAngle(double angle){
        motorTalonSRX.set(TalonSRXControlMode.MotionMagic, angle / TalonSRXConfiguration.degreePerTick);
    }

    public void testMotorApplyPower(double power) {
        motorTalonSRX.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void applyPower(double power) {
        motorTalonSRX.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void moveToAngle(double angle) {
        motorTalonSRX.set(TalonSRXControlMode.MotionMagic, ticksToDegrees(angle));
    }

    private double ticksToDegrees(double degrees) {
        return degrees / Swerve.TalonSRXConfiguration.degreePerTick;
    }

    

    private void configureMotionMagicAngle(Constants.Swerve.SwerveModuleConstants c) {

        // Disable motor safety so we can use hardware PID
        motorTalonSRX.setSafetyEnabled(false);

        motorTalonSRX.configNeutralDeadband(SRXAngle.NeutralDeadband, 30);

        motorTalonSRX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, SRXAngle.periodMs,  SRXAngle.timeoutMs);
        motorTalonSRX. setStatusFramePeriod(StatusFrame.Status_10_Targets, SRXAngle.periodMs,  SRXAngle.timeoutMs);

        motorTalonSRX.configPeakOutputForward(+1.0, SRXAngle.timeoutMs);
        motorTalonSRX.configPeakOutputReverse(-1.0, SRXAngle.timeoutMs);
        motorTalonSRX.configNominalOutputForward(0, SRXAngle.timeoutMs);
        motorTalonSRX.configNominalOutputReverse(0, SRXAngle.timeoutMs);

        /* FPID Gains */
        motorTalonSRX.selectProfileSlot(SRXAngle.SLOT_0, 0);
        motorTalonSRX.config_kP(SRXAngle.SLOT_0, SRXAngle.kP, SRXAngle.timeoutMs);
        motorTalonSRX.config_kI(SRXAngle.SLOT_0, SRXAngle.kI, SRXAngle.timeoutMs);
        motorTalonSRX.config_kD(SRXAngle.SLOT_0, SRXAngle.kD, SRXAngle.timeoutMs);
        motorTalonSRX.config_kF(SRXAngle.SLOT_0, SRXAngle.kF, SRXAngle.timeoutMs);

        motorTalonSRX.config_IntegralZone(SRXAngle.SLOT_0, SRXAngle.Izone,  SRXAngle.timeoutMs);
        motorTalonSRX.configClosedLoopPeakOutput(SRXAngle.SLOT_0, SRXAngle.PeakOutput, SRXAngle.timeoutMs);
        motorTalonSRX.configAllowableClosedloopError(SRXAngle.SLOT_0, SRXAngle.DefaultAcceptableError, SRXAngle.timeoutMs);
      
        motorTalonSRX.configClosedLoopPeriod(SRXAngle.SLOT_0, SRXAngle.closedLoopPeriod, SRXAngle.timeoutMs);

        motorTalonSRX.configMotionAcceleration(SRXAngle.Acceleration,SRXAngle.timeoutMs);
        motorTalonSRX.configMotionCruiseVelocity(SRXAngle.CruiseVelocity,SRXAngle.timeoutMs);
        motorTalonSRX.configMotionSCurveStrength(SRXAngle.Smoothing);
    }

    public void initQuadrature() { // Set absolute encoders
        int pulseWidth = motorTalonSRX.getSensorCollection().getPulseWidthPosition();

        if (TalonSRXConfiguration.kDiscontinuityPresent) {

            /* Calculate the center */
            int newCenter;
            newCenter = (TalonSRXConfiguration.kBookEnd_0 + TalonSRXConfiguration.kBookEnd_1) / 2;
            newCenter &= 0xFFF;

            /**
             * Apply the offset so the discontinuity is in the unused portion of
             * the sensor
             */
            pulseWidth -= newCenter;
        }
    }

    public void setEncoderforWheelCalibration(SwerveModuleConstants c) {
        double difference = getDriveAbsEncoder() - c.getAngleOffset()*4096.0/360.0;
        double encoderSetting = 0.0;

        // System.out.println("I0 d " + difference);

        if (difference < 0) {
            difference += TalonSRXConfiguration.clicksSRXPerFullRotation;
        }

        // System.out.println("I1 d " + difference);

        if (difference <= TalonSRXConfiguration.clicksSRXPerFullRotation / 2) {
            encoderSetting = difference;

        } else {
            encoderSetting = difference - TalonSRXConfiguration.clicksSRXPerFullRotation;
        }

        motorTalonSRX.setSelectedSensorPosition(encoderSetting);

        System.out.println("Set encoder for motor " + c.getAngleMotorID() + " to " + encoderSetting);

    }

    private void motorBrakeMode() {
        motorTalonSRX.setNeutralMode(NeutralMode.Brake);
    }

}
