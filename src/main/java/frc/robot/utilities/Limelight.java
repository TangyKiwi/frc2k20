package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight
{
    NetworkTable limelight;

    public static enum LED_MODE
    {
        ON, OFF, BLINKING;
    }

    public static enum PIPELINE_STATE
    {
        FRONT_VISION, BACK_VISION, FRONT_DRIVER, BACK_DRIVER;
    }

    public Limelight()
    {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getAngle()
    {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getArea()
    {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget()
    {
        return limelight.getEntry("tv").getBoolean(false);
    }

    public void setLED(LED_MODE newMode)
    {
       switch(newMode)
       {
           case ON: 
                limelight.getEntry("ledMode").setNumber(3);  
                break;
           case OFF:
                limelight.getEntry("ledMode").setNumber(1);  
                break; 
           case BLINKING:
                limelight.getEntry("ledMode").setNumber(2);  
               break;
       }
    }

    public void setPipeline(PIPELINE_STATE newPipeline)
    {
        switch(newPipeline){
            case FRONT_DRIVER: 
                limelight.getEntry("pipeline").setNumber(3);  
                break;
            case FRONT_VISION:
                limelight.getEntry("pipeline").setNumber(0);  
                break; 
            case BACK_DRIVER:
                limelight.getEntry("pipeline").setNumber(4);  
                break;
            case BACK_VISION:
                limelight.getEntry("pipeline").setNumber(1);  
                break;    
        }
    }
}