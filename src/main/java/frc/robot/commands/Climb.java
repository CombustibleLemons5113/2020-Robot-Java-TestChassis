package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {

    private Climber climber;
    
    public Climb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override //overrides current state of robot when class is initiated 
    public void initialize() {
        SmartDashboard.putString("ClimbStatus", "Currently Climbing!"); //Tells driver on an interface that the robot is climbing
    }

    @Override
    public void execute() {
        climber.setSpeed(kClimbSpeed); //supposed to set the speed of the climber
    }
}