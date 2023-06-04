package frc.robot.PassThroughSystems.Motor;

import frc.robot.Constants;

public interface BaseMotorInterface {
    
    void configureDriveMotor(Constants.Swerve.SwerveModuleConstants c);

    void configureAngleMotor(Constants.Swerve.SwerveModuleConstants c);
}
