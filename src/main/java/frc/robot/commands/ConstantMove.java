/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ConstantMove extends CommandBase {
    /**
     * Creates a new DriveCommands.
     */
    private DriveTrain driveTrain;

    public ConstantMove(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

	// Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //raise the hood here
        System.out.println("Error Initializing Constant Move");
        driveTrain.setLeftSpeed();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveTrain.setLeftSpeed();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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
