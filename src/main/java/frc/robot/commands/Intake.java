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
        this.intake = intake;
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
        if (indexer.getBallCount() < 5)
            intake.setSpeed(kClimbSpeed);
        else
            intake.stopIntake();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}