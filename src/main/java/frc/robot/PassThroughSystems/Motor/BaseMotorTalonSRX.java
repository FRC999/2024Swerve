package frc.robot.PassThroughSystems.Motor;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

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

    }

    public void configureAngleMotor(Constants.Swerve.SwerveModuleConstants c) {

        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isDriveMotorInverted());

        // Encoder configuration
        motorTalonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute,
                Constants.Swerve.TalonSRXConfiguration.kPIDLoopIdx,
                Constants.Swerve.TalonSRXConfiguration.configureTimeoutMs);
    }

    public double getDriveEncoderPosition() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getAngleEncoderPosition() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getDriveEncoderVelocity() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getAngleEncoderVelocity() {
        return motorTalonSRX.getSelectedSensorPosition();
    }

    public double getDriveEncoderPositionSI() {
        return motorTalonSRX.getSelectedSensorPosition()*Constants.Swerve.TalonSRXConfiguration.metersPerTick;
    }

    public double getAngleEncoderPositionSI() {
        return motorTalonSRX.getSelectedSensorPosition()*Constants.Swerve.TalonSRXConfiguration.degreePerTick;
    }

    public double getDriveEncoderVelocitySI() {
        return motorTalonSRX.getSelectedSensorPosition()*10.0*Constants.Swerve.TalonSRXConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
        return motorTalonSRX.getSelectedSensorPosition()*10.0*Constants.Swerve.TalonSRXConfiguration.degreePerTick;
    }
}
