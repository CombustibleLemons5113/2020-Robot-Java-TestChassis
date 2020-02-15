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

    @Override
    public void initialize() {
        SmartDashboard.putString("ClimbStatus", "Currently Climbing!");
    }

    @Override
    public void execute() {
        climber.setSpeed(kClimbSpeed);
    }
}