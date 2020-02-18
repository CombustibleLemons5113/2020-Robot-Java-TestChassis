package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    public WPI_TalonFX climber = new WPI_TalonFX(kClimber); //creates object for the TalonFx to instantiate as an object 

    public Climber () {
        climber.configFactoryDefault(); //sets the talons to factory default configuration
        climber.setNeutralMode(NeutralMode.Brake); //neutral throttle
        //do nothing
    }

    public void setSpeed(double speed) //sets the speed for the climber, has to be tested
    {
        climber.set(speed);
    }
}