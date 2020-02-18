package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class CenterTargetRobot extends CommandBase //this class is apart of the CommandBase 
{

    private LimeLight lime; 
    private DriveTrain driveTrain;

    //private double goal = 0.0;
    private double error;
    private double offsetX;
    private double margin = 0.2;

    private int finalCount = 0;

    private boolean SeesTarget = false;
    /*
    private double[][] gainTable = new double[][] {{5, 0.0005, 3},
                                                  {7.5, 0.001, 2.5} ,
                                                  {10, 0.0025, 2} ,
                                                  {12.5, 0.035, 1.5} ,
                                                  {15, 0.0065, 1} ,
                                                  {100, 0.01, 0.5}};
    */

    //private double[][] gainTable = new double [][] {{}};
    private int count = 0;
    private double offsetXPrev;

    private double OFFSET = -3;

    private double speed;
    //old is 0.0075
    private double kp = 0.0075   ;
    //old is 0.0005,0.0001,0.00075 is good
    private double ki = 0.00015;
    //old is 0.1    
    private double kd = 3;

    //good is 0.0075, 0.00015, 10


    private double deltaI;
    private double deltaD;

    public CenterTargetRobot(DriveTrain driveTrain, LimeLight lime)
    {
        this.driveTrain = driveTrain; // Using ‘this’ keyword to refer current class instance variables
        this.lime = lime;
        System.out.println("works");
        addRequirements(driveTrain);
        addRequirements(lime);
        offsetX = lime.getTx();
        deltaI = 0.2 * offsetX/Math.abs(offsetX);
        deltaD = 0;
        if(offsetX != 0)
        {
            SeesTarget = true;
        }
    }
    public double PID(double err)
    { 
        
        if(OFFSET == -3) {
            //System.out.println("I AM HERE SO IT BROKE");
            return deltaI;
        //return (kp * err) + (OFFSET * (err / Math.abs(err))) + (kd * deltaD);
        }

        return (kp * err) + (OFFSET * (err / Math.abs(err))) + (kd * deltaD);
    }
    public void updateI(double err) 
    {
        deltaI += err * ki;
    }
    public void updateD(double err) 
    {
        deltaD = offsetX - offsetXPrev;
    }   
    /*
    public int updatePandDa(double speed)
    {
        for(int i = 0; i < gainTable.length/3; i++)
        {
            if(speed < gainTable[i][0])
            {
                kp = gainTable[i][1];
                kd = gainTable[i][2];
                return 0;
            }
        }
        return 0;
    }
    */

    @Override
    public void execute() 
    {
        offsetX = lime.getTx();
        error = offsetX;
        finalCount++;
        if(Math.abs(offsetX) > margin)
        {
            count = 0;
            //System.out.println("Offset: "+OFFSET);
            // System.out.println(error + " error");
            //System.out.println(PID(error)+" output");
            //System.out.println(deltaI+" deltaI");
            driveTrain.driveCartesian(PID(error), -PID(error));
        }
        else
        {
            count += 1;
        }
        /*
        if(offsetX > 0 && Math.abs(offsetX) > 5)
        {
            Robot.driveTrain.driveCartesian(-PID(error), PID(error));
        }
        else if(offsetX < 0 && Math.abs(offsetX) > 5)
        {
            Robot.driveTrain.driveCartesian(PID(error), -PID(error));
        } 
        */

        
        offsetXPrev = offsetX;

        
        speed = Math.abs(driveTrain.getWheelSpeeds().leftMetersPerSecond);
        if(speed != 0)
        {
            //System.out.print("herenegjajgajsjfs");
            if(OFFSET == -3)
            {
                OFFSET = Math.abs(PID(error));
            }
            deltaI = 0;
        }
        /*
        if (Math.abs(error) < Math.abs(errorInit*0.5) && !justSwitched) {
            justSwitched = true;
            OFFSET *= 0.9;
        }
        */
        updateI(error);
        updateD(error);

        //updatePandDa(Math.abs(Robot.driveTrain.getSpeeds().leftMetersPerSecond));
    }
    @Override
    public boolean isFinished() {
        //if
        //System.out.println(finalCount);
        if (finalCount > 1000) {
            finalCount = 0;
            return true;
        }
        else if(SeesTarget) {
            if (count > 10) {
                finalCount = 0;
                return true;
            }
            return count > 10;
        }
        else {
            finalCount = 0;
            return true;
        }
    }
}