package frc.robot.PassThroughSystems.Motor;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.BaseMotorControllerTypes;
import frc.robot.Constants.Swerve.SwerveModuleConstants;

public class BaseMotorPassthrough implements BaseMotorInterface{
    private BaseMotorInterface baseMotorInterface; //downcasting to the individual motor types
  
    /** Creates a new IMUSubsystem. */
    public BaseMotorPassthrough(BaseMotorControllerTypes motorType, int CANID) {

        switch(motorType) {
            case TALON_SRX:
              baseMotorInterface = new BaseMotorTalonSRX(CANID);
              break;
            default:
              
          }
    }

    public void configureDriveMotor(Constants.Swerve.SwerveModuleConstants c){
      baseMotorInterface.configureDriveMotor(c);
    }

    public void configureAngleMotor(Constants.Swerve.SwerveModuleConstants c){
      baseMotorInterface.configureAngleMotor(c);
    }
    
}
