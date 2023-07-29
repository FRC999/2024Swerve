package frc.robot.Devices;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.OIConstants.ControllerDeviceType;

public class Controller extends Joystick {

    ControllerDevice cd;
    ControllerDeviceType cdt;
    double dx; // X deadband for performance
    double dy; // Y deadband for performance
    double dm; // Omega deadband for performance
    boolean cubeControllerLeftStick; // If false use linear transformation from joystick input to power
                            // If true use cube transformation
                            // This concept was taken from nu23 of team 125 code.
    boolean cubeControllerRightStick;

    // The next three variables will contain cube values of deadbands used for cube driving.
    // This is done for performance reasons to reduce periodic calculations for cube driving.          
    private double cubeDeadbandX;
    private double cubeDeadbandY;
    private double cubeDeadbandOmega;
    
    public Controller(Constants.OIConstants.ControllerDevice cd) {
        super(cd.getPortNumber()); // This needs to be done because the joystick parent class has a non-default constructor
        this.cd = cd;
        this.cdt = cd.getControllerDeviceType();  // This is done so the controller type is not 
                                                  // re-evaluated every time you need to get axis value
        this.dx= cd.getDeadbandX();
        this.dy= cd.getDeadbandY();
        this.dm= cd.getDeadbandOmega();

        this.cubeControllerLeftStick=cd.isCubeControllerLeftStick();
        this.cubeControllerRightStick=cd.isCubeControllerRightStick();

        this.cubeDeadbandX = dx*dx*dx;
        this.cubeDeadbandY = dy*dy*dy;
        this.cubeDeadbandOmega = dm*dm*dm;
    } 

    /**
     * Controllers with a single joystick will only return left joystick values.
     */

    public double getLeftStickY() {
        double rawY = this.getY();
        double result;

        switch(cdt){
            case LOGITECH:
                if (this.cubeControllerLeftStick) {
                    double cubeY = rawY*rawY*rawY;
                    result = (cubeY - (rawY > 0 ? 1 : -1) * cubeDeadbandY)/(1 - cubeDeadbandY); // cubeController
                    result = Math.abs(result) > this.cubeDeadbandY ? result : 0; // Ignores deadband values
                } else {
                    result = (MathUtil.applyDeadband(rawY, dy)); // linear
                }
                return result;
        }
        return 0; // Unknown controller type
    }

    public double getLeftStickX() {
        double rawX = this.getX();
        double result;

        switch(cdt){
            case LOGITECH:
                if (this.cubeControllerLeftStick) {
                    double cubeX = rawX*rawX*rawX;
                    result = (cubeX - (rawX > 0 ? 1 : -1) * cubeDeadbandX)/(1 - cubeDeadbandX); // cubeController
                    result = Math.abs(result) > this.cubeDeadbandX ? result : 0; // Ignores deadband values
                } else {
                    result = (MathUtil.applyDeadband(rawX, dx)); // linear
                }
                return result;
        }
        return 0; // Unknown controller type
    }

    public double getLeftStickOmega() {
        double rawOmega = this.getTwist();
        double result;

        switch(cdt){
            case LOGITECH:
                if (this.cubeControllerRightStick) {
                    double cubeOmega = rawOmega*rawOmega*rawOmega;
                    result = (cubeOmega - (rawOmega > 0 ? 1 : -1) * cubeDeadbandOmega)/(1 - cubeDeadbandOmega); // cubeController
                    result = Math.abs(result) > this.cubeDeadbandOmega ? result : 0; // Ignores deadband values
                } else {
                    result = (MathUtil.applyDeadband(rawOmega, dm)); // linear
                }
                return result;
        }
        return 0; // Unknown controller type
    }



}
