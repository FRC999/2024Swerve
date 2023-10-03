package frc.robot.PassThroughSystems.Motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsForSwerveModules.NEOAngle;
import frc.robot.Constants.PIDConstantsForSwerveModules.SRXAngle;
import frc.robot.Constants.SwerveChassis.NEOSwerveConfiguration;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstants;

/*
 * This is a specific base motor implementation for the motors connected to SparkMAX NEOs
 */
public class BaseMotorNEO implements BaseMotorInterface {
    private CANSparkMax motorNEO;
    private int CANID;
    private SwerveModuleConstants cAngle;

    public BaseMotorNEO(int CANID) {
        System.out.println("**** Activating SparkMAX NEO CANID:" + CANID);

        motorNEO = new CANSparkMax(CANID, MotorType.kBrushless);

        this.CANID=CANID;

    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {
        //TODO: Add logic to configureDriveMotor

        motorBrakeMode();
    }

    public void configureAngleMotor(SwerveModuleConstants c) {

        this.cAngle=c;

        //TODO: Add logic to configureAngleMotor

        // Configure PID values

        motorNEO.getPIDController().setP(NEOAngle.kP);
        motorNEO.getPIDController().setI(NEOAngle.kI);
        motorNEO.getPIDController().setD(NEOAngle.kD);

        motorNEO.setCANTimeout(0);
        motorBrakeMode();

        // if (CANID == 1) {
        //     setAngleMotorChassisAngleSI(0);
        // }
        //setAngleMotorChassisAngleSI(0);
    }

    public double getDriveEncoderPosition() {
        return motorNEO.getEncoder().getPosition();
    }

    public double getAngleEncoderPosition() {
        return motorNEO.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public double getDriveEncoderVelocity() {
        return motorNEO.getEncoder().getVelocity();
    }

    public double getAngleEncoderVelocity() {
        return motorNEO.getAbsoluteEncoder(Type.kDutyCycle).getVelocity();
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
        motorNEO.getPIDController().setReference(degreesToTicks(angle), ControlType.kPosition);
        
    }

    public void testMotorApplyPower(double power) {
        motorNEO.set(power);
    }

    public void applyPower(double power) {
        motorNEO.set(power);
    }

    private void motorBrakeMode(){
        motorNEO.setIdleMode(IdleMode.kBrake);
    }

    private double degreesToTicks(double degrees) {
        return ((degrees+cAngle.getAngleOffset()) / NEOSwerveConfiguration.degreePerTick)  
         % 
        (NEOSwerveConfiguration.ticksPerFullRotation);
    }


}
