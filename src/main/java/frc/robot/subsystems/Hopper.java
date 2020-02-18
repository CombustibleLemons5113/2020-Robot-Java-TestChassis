package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.HopperConstants.*;

public class Hopper extends SubsystemBase {

    private CANSparkMax hopperMotor;
    private CANEncoder hopperEncoder;
    //private DoubleSolenoid intakePistons = new DoubleSolenoid(kLIntakePiston, kRIntakePiston);

    public Hopper() {
        hopperMotor = new CANSparkMax(kHopperMotor, MotorType.kBrushless);
        hopperEncoder = new CANEncoder(hopperMotor);
        hopperMotor.restoreFactoryDefaults();
        hopperMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setSpeed(double speed) {
        hopperMotor.set(speed);
    }

    public void stopIntake() {
        this.setSpeed(0);
    }


}