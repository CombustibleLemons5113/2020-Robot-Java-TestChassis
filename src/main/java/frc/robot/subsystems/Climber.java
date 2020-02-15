package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    public WPI_TalonFX climber = new WPI_TalonFX(kClimber);

    public Climber () {
        climber.configFactoryDefault();
        climber.setNeutralMode(NeutralMode.Brake);
        //do nothing
    }

    public void setSpeed(double speed)
    {
        climber.set(speed);
    }
}