package frc.robot.PassThroughSystems.Motor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class BaseMotorTalonSRX implements BaseMotorInterface {
    private WPI_TalonSRX motorTalonSRX;   

    public BaseMotorTalonSRX(int CANID){
        motorTalonSRX = new WPI_TalonSRX(CANID);
    }
}
