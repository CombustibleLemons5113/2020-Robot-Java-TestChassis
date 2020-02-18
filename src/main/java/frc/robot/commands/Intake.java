package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intaker;
public class Intake extends CommandBase {

    private Intaker intake;
    private Indexer indexer;
    
    public Intake(Intaker intake, Indexer indexer) {
        this.intake = intake; // Using ‘this’ keyword to refer current class instance variables
        this.indexer = indexer;
        addRequirements(intake);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ClimbStatus", "Currently Climbing!");
    }

    @Override
    public void execute() {
        if (indexer.getBallCount() < 5) //if the indexer reads less than 5 balls, than the intake will continue to spin
            intake.setSpeed(kClimbSpeed);
        else
            intake.stopIntake(); //stops intake if ball count is 5
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            intake.stopIntake(); //If anything happens, such as a manual overide, then Intake is stopped
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}