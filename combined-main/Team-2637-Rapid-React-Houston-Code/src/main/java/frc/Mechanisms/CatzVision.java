package frc.Mechanisms;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatzVision 
{
    public boolean hasValidTarget;
    public boolean inShootingRange;

    public double xErrorOffsetDeg;    //Positive value = target is to the right, Negative value = target is to the left
    public double yErrorOffsetDeg;
    public double targetPresent;

    private final double LIMELIGHT_MOUNT_HEIGHT = 45.5;

    private final double LIMELIGHT_MOUNT_ANGLE = 31.0;

    private final double HUB_TARGET_HEIGHT_TOP = 100.0;

    private double angleToTargetDeg;
    private double angleToTargetRad;
    private double distanceToTargetInch;

    public CatzVision()
    {
        NetworkTableInstance.getDefault().getTable("limelight-drvcam").getEntry("camMode").setNumber(1.0);
        NetworkTableInstance.getDefault().getTable("limelight-drvcam").getEntry("ledMode").setNumber(1.0);
    }

    public void turretTracking()
    {
        targetPresent    = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        xErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

        if (targetPresent == 1.0)
        {
            hasValidTarget = true;    
        }
        else
        {
            hasValidTarget  = false;
        }
        
        smartDashboardVision();
    }

    public double getDistanceToTarget()
    {
        
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

        angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE + yErrorOffsetDeg;
        angleToTargetRad = angleToTargetDeg * (Math.PI / 180); //Convert angle from degrees to radians
        distanceToTargetInch = (HUB_TARGET_HEIGHT_TOP - LIMELIGHT_MOUNT_HEIGHT) / Math.tan(angleToTargetRad);

        return distanceToTargetInch;
    }

    public double getXErrorOffset()
    {
        xErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return xErrorOffsetDeg - 3.28; //offset because turret aims too much to right
    }

    public double getYErrorOffset()
    {
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return yErrorOffsetDeg;
    }

    public void smartDashboardVision()
    {
        SmartDashboard.putBoolean("Has Valid Target", hasValidTarget);
        SmartDashboard.putNumber("X Offset", xErrorOffsetDeg);
        SmartDashboard.putNumber("Y Offset", yErrorOffsetDeg);
        SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    }

    public boolean hasValidTarget()
    {
        return hasValidTarget;
    }
}
