package frc.robot.PassThroughSystems.Motor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class BaseMotorTalonSRX implements BaseMotorInterface {
    private WPI_TalonSRX motorTalonSRX;

    public BaseMotorTalonSRX(int CANID) {
        motorTalonSRX = new WPI_TalonSRX(CANID);
    }

    public void configureDriveMotor(Constants.Swerve.SwerveModuleConstants c) {

        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isDriveMotorInverted());
    }

    public void configureAngleMotor(Constants.Swerve.SwerveModuleConstants c) {
        
        motorTalonSRX.configFactoryDefault();
        motorTalonSRX.setInverted(c.isDriveMotorInverted());
    }
}
