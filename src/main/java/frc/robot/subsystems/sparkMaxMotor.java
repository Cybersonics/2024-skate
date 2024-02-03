package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sparkMaxMotor extends SubsystemBase {

    private static sparkMaxMotor instance;
    private CANSparkMax motor;

    public sparkMaxMotor(int idCAN) {
        motor = new CANSparkMax(idCAN, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        // motor.setInverted(invertDrive);// setInverted reverses the both the motor and
        // the encoder direction.
        // motor.setOpenLoopRampRate(RAMP_RATE);// This provides a motor ramp up time to
        // prevent brown outs.
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(30);
    }

    public static sparkMaxMotor getInstance(int idCAN) {
        if(instance == null) {
            instance = new sparkMaxMotor(idCAN);
        }
        return instance;
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    private boolean isCoastMode = false;
	public boolean toggleMode() {
		return isCoastMode;
	}
}
