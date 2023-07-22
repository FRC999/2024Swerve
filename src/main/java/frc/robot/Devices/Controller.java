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


    public Controller(Constants.OIConstants.ControllerDevice cd) {
        super(cd.getPortNumber()); // This needs to be done because the joystick parent class has a non-default constructor
        this.cd = cd;
        this.cdt = cd.getControllerDeviceType();  // This is done so the controller type is not 
                                                  // re-evaluated every time you need to get axis value
        this.dx= cd.getDeadbandX();
        this.dy= cd.getDeadbandY();
        this.dm= cd.getDeadbandOmega();
    } 

    /**
     * Controllers with a single joystick will only return left joystick values.
     */

    public double getLeftStickY() {
        switch(cdt){
            case LOGITECH:
                return (MathUtil.applyDeadband(this.getY(), dy));
        }
        return 0; // Unknown controller type
    }

    public double getLeftStickX() {
        switch(cdt){
            case LOGITECH:
                return (MathUtil.applyDeadband(this.getX(), dx));
        }
        return 0; // Unknown controller type
    }

    public double getLeftStickOmega() {
        switch(cdt){
            case LOGITECH: 
                return (MathUtil.applyDeadband(this.getTwist(), dm));
        }
        return 0; // Unknown controller type
    }


}
