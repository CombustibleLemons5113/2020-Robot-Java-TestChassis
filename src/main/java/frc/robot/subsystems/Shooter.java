package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    /*
    Creates a new Shooter
    */

    private WPI_TalonFX leftShooter = new WPI_TalonFX(Constants.kLShooter);
    private WPI_TalonFX rightShooter = new WPI_TalonFX(Constants.kRShooter);

    public Shooter() {

        leftShooter.follow(rightShooter);

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);

        leftShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        leftShooter.setNeutralMode(NeutralMode.Brake);
        rightShooter.setNeutralMode(NeutralMode.Brake);

    }

    public void setSpeeds(double speed) {
        rightShooter.set(speed);
    }

}