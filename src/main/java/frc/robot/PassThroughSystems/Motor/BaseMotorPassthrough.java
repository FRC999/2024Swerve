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
}
