package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private WPI_TalonFX shooterSlave = new WPI_TalonFX(kLShooterMotor);
    private WPI_TalonFX shooterMaster = new WPI_TalonFX(kRShooterMotor);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kSShooter, kVShooter);

    //instantiate the hood here
    //private DoubleSolenoid hood = new DoubleSolenoid(kRHoodPiston, kLHoodPiston);

    public Shooter()
    {
        shooterMaster.configFactoryDefault();
        shooterSlave.configFactoryDefault();

        shooterSlave.set(ControlMode.Follower, shooterMaster.getDeviceID());
        

        //Sets up PID for the right shooter configuration
		shooterMaster.config_kP(1, kShooterkP);
        shooterMaster.config_kI(0, kShooterkI);
        shooterMaster.config_kD(0, kShooterkD);
        shooterMaster.config_kF(0, feedforward.calculate(500));

        //determines if the values are inverted or not 
        shooterSlave.setInverted(true);
        shooterMaster.setInverted(false);

        //gives whatever chosen feedback to the user 
        shooterSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        //sets the shooters in a neutral state
        shooterSlave.setNeutralMode(NeutralMode.Coast); 
        shooterMaster.setNeutralMode(NeutralMode.Coast);

    }

    public void setSpeed(double speed) 
    {
        shooterMaster.set(speed);
        //SmartDashboard.putNumber("Velocity: ", shooterMaster.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("Current:", shooterMaster.getSupplyCurrent());
    }

    public double getRPM() 
    {
        return shooterMaster.getSelectedSensorVelocity();
    }

    public void hoodUp() {
        //hood.set(Value.kForward);
    }

    public void hoodDown() {
        //hood.set(Value.kReverse);
    }

    public boolean getPistonState() {
        return true; //(hood.get() == DoubleSolenoid.Value.kForward);
    }

    protected void useOutput(double output, double setpoint) {
        shooterMaster.setVoltage(output + feedforward.calculate(setpoint));

    }

    /*@Override
    protected double getMeasurement() {
        return rightShooter.getSelectedSensorVelocity();
    }

    /*/

}
