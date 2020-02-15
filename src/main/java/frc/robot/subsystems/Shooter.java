package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase
{
    /*
    //Shooter PID Values
    private double kp;
    private double ki;
    private double kd;

    //Keep track of sum error and change in error
    private double sumError;
    private double changeError;
    */

    /*
    Creates a new Shooter
    */

    private WPI_TalonFX leftShooter = new WPI_TalonFX(kLShooterMotor);
    private WPI_TalonFX rightShooter = new WPI_TalonFX(kRShooterMotor);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kSShooter, kVShooter);

    //instantiate the hood here
    private DoubleSolenoid hood = new DoubleSolenoid(kRHoodPiston, kLHoodPiston);

    public Shooter()
    {
        leftShooter.follow(rightShooter);

		rightShooter.config_kP(0, kShooterkP);
        rightShooter.config_kI(0, kShooterkI);
        rightShooter.config_kD(0, kShooterkD);
        rightShooter.config_kF(0, feedforward.calculate(500));

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);

        leftShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        leftShooter.setNeutralMode(NeutralMode.Brake);
        rightShooter.setNeutralMode(NeutralMode.Brake);

    }

    public void setSpeed(double speed) 
    {
        rightShooter.set(ControlMode.Velocity, speed);
    }

    public double getRPM() 
    {
        return rightShooter.getSelectedSensorVelocity();
    }

    public void hoodUp() {
        hood.set(Value.kForward);
    }

    public void hoodDown() {
        hood.set(Value.kReverse);
    }

    public boolean getPistonState() {
        return (hood.get() == DoubleSolenoid.Value.kForward);
    }

    protected void useOutput(double output, double setpoint) {
        rightShooter.setVoltage(output + feedforward.calculate(setpoint));

    }

    /*@Override
    protected double getMeasurement() {
        return rightShooter.getSelectedSensorVelocity();
    }

    /*/

}
