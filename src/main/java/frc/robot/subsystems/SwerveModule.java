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

    public SwerveModule(int moduleNumber, Constants.Swerve.SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.getAngleOffset();

        driveMotor = new BaseMotorPassthrough(moduleConstants.getDriveMotorControllerType(), moduleConstants.getDriveMotorID());

        angleMotor = new BaseMotorPassthrough(moduleConstants.getAngleMotorControllerType(), moduleConstants.getAngleMotorID());


   
    

    }
}
