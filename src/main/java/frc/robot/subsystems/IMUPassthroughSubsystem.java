package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PassThroughSystems.IMU.IMUInterface;
import frc.robot.PassThroughSystems.IMU.IMUNavX;
import frc.robot.PassThroughSystems.IMU.IMUPigeon2;

/*
 * The IMU object should be instantiated from this class. The class has a selector that will
 * create additional hardware-specific objects. However, other subsystems and commands will
 * use this passthrough class, which exposes "standard" IMU methods that should be implemented
 * by all hardware-specific classes. That way the non-IMU code does not need to know which IMU
 * is actually used.
 * 
 * Since IMU represents a separate hardware component, it's defined as a subsystem
 * (via "extends SubsystemBase"). It means that you can add "periodic" code to it.
 * We do not currently need that. However, when used in a competition, you might find it necessary to do so.
 */

public class IMUPassthroughSubsystem extends SubsystemBase implements IMUInterface {

  private IMUInterface imu; // We will use downcasting to set this - it will point to methods either in NavX
  // or Pigeon subsystems

  /** Creates a new IMUSubsystem. 
   * This is an IMU Passthrough class, meaning, it provides a class for the generic IMU instantiation.
   * The "switch" implementation chooser below will instantiate hardware-specific implementation, and will
   * map "imu" variable to that object.
   * That means all hardware-specific implementations must implement the same methods, which is enforced
   * via interface IMUInterface. That interface contains all methods that must be declared as "public"
   * both in this class and in the hardware-specific classes. All other methods (if they're needed) should be private.
  */
  public IMUPassthroughSubsystem() {

    /*
     * Depending on the IMU type specified in Constants, the variable "imu" will point to the instance
     * of the class specific to the hardware you have, e.g. Pigeon2 or NavX
     * All such implementation classes must have public methods specified by the IMUInterface
     */
    switch (Constants.IMUConstants.imuType) {
      case Pigeon2:
        imu = new IMUPigeon2();
        break;
      case NavX:
        imu = new IMUNavX();
        break;
      default:
    }

    imu.zeroYaw(); // TODO: At start of game, robot must be pointed towards opposite team's side
                   // (This is our zero value). We do not know which side red/blue is on



  }

  public double getPitch() {
    return imu.getPitch();
  }

  public double getRoll() {
    return imu.getRoll();
  }

  public double getYaw() {
    return imu.getYaw();
  }

  public double zeroYaw() {
    return imu.zeroYaw();
  }

  public double setYaw(double y) {
    return imu.setYaw(y);
  }

  public double getTurnRate() {
    return imu.getTurnRate();
  }

  public Rotation2d getHeading() {
    return imu.getHeading();
  }

}
