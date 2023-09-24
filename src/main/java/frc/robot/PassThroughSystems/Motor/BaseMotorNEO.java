package frc.robot.PassThroughSystems.Motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsForSwerveModules.SRXAngle;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstants;

/*
 * This is a specific base motor implementation for the motors connected to SparkMAX NEOs
 */
public class BaseMotorNEO implements BaseMotorInterface {
    private CANSparkMax motorNEO;

    public BaseMotorNEO(int CANID) {
        System.out.println("**** Activating SparkMAX NEO CANID:" + CANID);

        motorNEO = new CANSparkMax(CANID, MotorType.kBrushless);
    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {
        //TODO: Add logic to configureDriveMotor

        motorBrakeMode();
    }

    public void configureAngleMotor(SwerveModuleConstants c) {
        //TODO: Add logic to configureAngleMotor

        motorBrakeMode();
    }

    public double getDriveEncoderPosition() {
        return motorNEO.getEncoder().getPosition();
    }

    public double getAngleEncoderPosition() {
        return motorNEO.getEncoder().getPosition();
    }

    public double getDriveEncoderVelocity() {
        return motorNEO.getEncoder().getVelocity();
    }

    public double getAngleEncoderVelocity() {
        return motorNEO.getEncoder().getVelocity();
    }

    public double getDriveEncoderPositionSI() {
       
        return getDriveEncoderPosition()*Constants.SwerveChassis.NEOSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderPositionSI() {
      
        return getAngleEncoderPosition()*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public double getDriveEncoderVelocitySI() {
   
        return getDriveEncoderVelocity()*Constants.SwerveChassis.NEOSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
       
        return getAngleEncoderVelocity()*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public void setAngleMotorChassisAngleSI(double angle) {
        motorNEO.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void testMotorApplyPower(double power) {
        motorNEO.set(power);
    }

    public void applyPower(double power) {
        motorNEO.set(power);
    }

    private void brakeMode(){
        motorNEO.setIdleMode(CANSparkMax.IdleMode brake);
    }


}
