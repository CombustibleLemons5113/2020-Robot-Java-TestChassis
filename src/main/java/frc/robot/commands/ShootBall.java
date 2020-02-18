/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootBall extends CommandBase {
    /**
     * Creates a new DriveCommands.
     */
    public int spinCount = 0;
    public int hoodCount = 0;
    public double speed;

    private final Shooter shooter;

    public ShootBall(double speed, Shooter shooter) {
        System.out.print("heleleleleo");
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    public double calculateNeededSpeed () { //method to figure the speed of the shooter
        //double distance = Robot.m_robotContainer.lime.getDistanceToTarget();
        //there is literally no way to do that without testing it using the actual shooter
        return speed;
    }

	// Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //raise the hood here
        shooter.hoodUp();
        shooter.setSpeed(calculateNeededSpeed());
        //Robot.shooter.switchPistonState();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.print("helelelelelelelellelelelo");
        if(true)
        {
            spinCount++;
            //Robot.shooter.setSpeed(speed.getAsDouble());
            shooter.setSpeed(speed);
        }
        else
            hoodCount++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        shooter.hoodDown();
        //Lower the hood here
        //Robot.shooter.switchPistonState();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
