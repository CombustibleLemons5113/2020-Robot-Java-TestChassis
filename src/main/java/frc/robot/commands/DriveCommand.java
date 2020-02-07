/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommands.
   */

   private DoubleSupplier leftVal;
   private DoubleSupplier rightVal;
  public DriveCommand(DoubleSupplier leftVal, DoubleSupplier rightVal) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(Robot.driveTrain);
    this.leftVal = leftVal;
    this.rightVal = rightVal;
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("L: " + leftVal.getAsDouble());
    //System.out.println("R: " + rightVal.getAsDouble() + "\n");
    Robot.driveTrain.driveCartesian(leftVal.getAsDouble(), rightVal.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
