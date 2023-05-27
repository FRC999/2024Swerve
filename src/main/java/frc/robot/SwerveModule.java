package frc.robot;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveModule<T extends Object> {
    
    private int moduleNumber;
    private double angleOffset;
    private T wmConstants;

    public SwerveModule(int moduleNumber, T wheelModule) {
        this.moduleNumber = moduleNumber;
        this.wmConstants = wheelModule;
        //this.angleOffset = wmConstants.angleOffset;
        double a = wmConstants.getAngleMotorID;


    }
}
