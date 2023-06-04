package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.SwerveModuleConstants;
import frc.robot.PassThroughSystems.Motor.BaseMotorPassthrough;


public class SwerveModule extends SubsystemBase {

    private BaseMotorPassthrough driveMotor;
    private BaseMotorPassthrough angleMotor;

    private int moduleNumber;
    private double angleOffset;

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


   
    

    }
}
