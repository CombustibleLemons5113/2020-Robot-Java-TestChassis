
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

    /**
        * Handler for the Limelight on the robot:
        * Capabilites include:
            * Getter methods for tx, anything else can be added
            * Updates all values in limelight
    **/

    NetworkTable table;
    NetworkTableEntry tx, ty, ta;
    double offsetX;
    double kp = 0.1;
    public LimeLight()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }
    public void update()
    {
        //System.out.println(table);
        
        
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        offsetX = x;
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    /*
    public double getLeft()
    {
        update();
        if(offsetX > 0 && Math.abs(offsetX) > 5)
        {
            return -kp*(tx.getDouble(0.0));
        }
        else if(offsetX < 0 && Math.abs(offsetX) > 5)
        {
            return kp*(tx.getDouble(0.0));
        } 
        return 0;
    }
    public double getRight()
    {
        update();
        if(offsetX > 0 && Math.abs(offsetX) > 5)
        {
            return kp*(tx.getDouble(0.0));
        }
        else if(offsetX < 0 && Math.abs(offsetX) > 5)
        {
            return -kp*(tx.getDouble(0.0));
        } 
        return 0;
    }
    */
    
    public double getTx()
    {
        update();
        return tx.getDouble(0.0);
    }
}