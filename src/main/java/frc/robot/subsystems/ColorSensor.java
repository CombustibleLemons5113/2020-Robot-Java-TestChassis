package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {

    private ColorSensorV3 color;
    
    public ColorSensor()  { 
        color = new ColorSensorV3(I2C.Port.kMXP); //I2C port goes here.
    }

    public Color getColor() {
       return color.getColor(); //"returns most likely color"
    }

    public int getRed() {
        return color.getRed();
    }

    public int getBlue() {
        return color.getBlue();
    }

    public int getGreen() {
        return color.getGreen();
    }

    public void getValues(){
        System.out.print("R: " + getRed() + " G: " + getGreen() + "B: " + getBlue());

    }

    /* According to the Game Manual, the CMYB values for the colors, and RBG conversions for them, are as follows:
    Blue: C: 100 M: 0 Y: 0 B: 0 / R: 0 G: 255 B: 255
    Green: C: 100 M: 0 Y: 100 B: 0 / R: 0 G: 255 B: 0
    Red: C: 0 M: 100 Y: 100 B: 0 / R: 255 G: 0 B: 0
    Yellow: C: 0 M: 0 Y: 100 B: 0 / R: 255 G: 255 B: 0
    */

}