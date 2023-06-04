package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PassThroughSystems.IMU.IMUInterface;
import frc.robot.PassThroughSystems.IMU.IMUNavX;
import frc.robot.PassThroughSystems.IMU.IMUPigeon2;

public class IMUPassthroughSubsystem extends SubsystemBase implements IMUInterface {
private IMUInterface imu; // We will use downcasting to set this - it will point to methods either in NavX
  // or Pigeon subsystems

  /** Creates a new IMUSubsystem. */
  public IMUPassthroughSubsystem() {

      // if (Constants.RobotProperties.isIMU) { // If IMU should be present - set it
      // in Robot.java for each bot type
      // if (Constants.RobotProperties.isNaVX) {
      // imu = new NavXIMUSubsystem();
      // } else {
      // imu = new PigeonIMUSubsystem();
      // }
      // }
      // }

      switch (Constants.IMUConstants.imuType) {
          case Pigeon2:
              imu = new IMUPigeon2();
              break;
          case NavX:
              imu = new IMUNavX();
              break;
          default:

      }

      imu.zeroYaw(); //TODO: At start of game, robot must be pointed towards opposite team's side (This is our zero value). We do not know which side red/blue is on
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
