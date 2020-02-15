package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intaker extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(kIntakeMotor, MotorType.kBrushless);
    private DoubleSolenoid intakePistons = new DoubleSolenoid(kLIntakePiston, kRIntakePiston);

    public Intaker() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void intakePistonsIn() {
        intakePistons.set(Value.kReverse);
    }

    public void intakePistonsOut() {
        intakePistons.set(Value.kForward);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        setSpeed(0);
    }


}