package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sparkFlexMotor extends SubsystemBase {

    private static sparkFlexMotor instance;
    private CANSparkFlex motorOne;    
    private CANSparkFlex motorTwo;
    
    public sparkFlexMotor(int idCANOne) {
        motorOne = new CANSparkFlex(idCANOne, MotorType.kBrushless);        
        motorOne.restoreFactoryDefaults();
        motorOne.setIdleMode(IdleMode.kCoast);
        // motorOne.setInverted(invertDrive);
        // motorOne.setSmartCurrentLimit(40);
    }

    public sparkFlexMotor(int idCANOne, int idCANTwo) {
        motorOne = new CANSparkFlex(idCANOne, MotorType.kBrushless);        
        motorOne.restoreFactoryDefaults();
        motorOne.setIdleMode(IdleMode.kCoast);
        // motorOne.setInverted(invertDrive);
        // motorOne.setSmartCurrentLimit(40);

        motorTwo = new CANSparkFlex(idCANTwo, MotorType.kBrushless);
        motorTwo.restoreFactoryDefaults();
        motorTwo.setIdleMode(IdleMode.kCoast);
        // motorTwo.setInverted(invertDrive);
        // motorTwo.setSmartCurrentLimit(40);
    }

    public static sparkFlexMotor getInstance(int idCANOne, int idCANTwo) {
        if(instance == null) {
            instance = new sparkFlexMotor(idCANOne, idCANTwo);
        }
        return instance;
    }

    public void setSpeed(double speed) {
        motorOne.set(speed); 
        if(motorTwo != null) {       
            motorTwo.set(speed);
        }
    }

    private boolean isCoastMode = false;
	public boolean toggleMode() {
		return isCoastMode;
	}
}
