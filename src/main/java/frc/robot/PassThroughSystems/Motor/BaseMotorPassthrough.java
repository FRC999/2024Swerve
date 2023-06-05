package frc.robot.PassThroughSystems.Motor;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.BaseMotorControllerTypes;
import frc.robot.Constants.Swerve.SwerveModuleConstants;

/*
 * The motor object should be instantiated from this class. The class has a selector that will
 * create additional hardware-specific objects. However, other subsystems and commands will
 * use this passthrough class, which exposes "standard" motor-related methods that should be implemented
 * by all hardware-specific classes. That way the non-motor-specific code does not need to know which
 * motor type is actually used.
 * 
 * Since we often want encoders directly connected to the motor controllers in order to use
 * hardware PID loops, we expose encoders through the motor object, rather than a separate encoder object.
 * However, if desired, feel free to make the code even more generic by separating encoders into a
 * subsystem.
 * 
 * While technically this class is not really needed (if you move the "switch" motor type selector logic to the
 * class that needs to instantiate motors, such as SwerveModule), this class allows separating individual
 * motor logic from the Swerve logic, albeit at the expense of the additional objects and extra calls.
 * 
 * And extra call occurs because the SwerveModule would call a method of this class when it wants it to
 * do something, and this class' method will call a method in a specific implementation with the same name.
 * E.g. a call to the "configureDriveMotor" of this class will simply result in calling "configureDriveMotor"
 * of specific implementation, such as the same method in BaseMotorTalonSRX.
 */
public class BaseMotorPassthrough implements BaseMotorInterface {
  private BaseMotorInterface baseMotorInterface; // downcasting to the individual motor types

  /** Creates a new IMUSubsystem. */
  public BaseMotorPassthrough(BaseMotorControllerTypes motorType, int CANID) {

    switch (motorType) {
      case TALON_SRX:
        baseMotorInterface = new BaseMotorTalonSRX(CANID);
        break;
      default:

    }
  }

  public void configureDriveMotor(Constants.Swerve.SwerveModuleConstants c) {
    baseMotorInterface.configureDriveMotor(c);
  }

  public void configureAngleMotor(Constants.Swerve.SwerveModuleConstants c) {
    baseMotorInterface.configureAngleMotor(c);
  }

}
