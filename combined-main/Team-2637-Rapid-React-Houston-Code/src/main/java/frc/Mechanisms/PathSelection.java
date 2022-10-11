package frc.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

/***************************************************************************
    *
    * Autonomous selections
    * 
***************************************************************************/
public class PathSelection
{  
    private final int LEFT_POS = 1;
    private final int MIDDLE_POS_2_CARGO = 2;
    private final int MIDDLE_POS_4_CARGO = 3;
    private final int RIGHT_POS = 4;

    private final SendableChooser<Boolean> chosenAllianceColor = new SendableChooser<>();
    private final SendableChooser<Integer> chosenPath = new SendableChooser<>();

    
    public void initializePathOptions()
    {
        chosenAllianceColor.setDefaultOption("Blue Alliance", Robot.constants.BLUE_ALLIANCE);
        chosenAllianceColor.addOption("Red Alliance", Robot.constants.RED_ALLIANCE);
        SmartDashboard.putData(Robot.constants.ALLIANCE_COLOR, chosenAllianceColor);

        chosenPath.setDefaultOption(Robot.constants.POSITION_SELECTOR1, 1);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR2, 2);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR3, 3);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR4, 4);
        SmartDashboard.putData(Robot.constants.ALLIANCE_POSITION, chosenPath);
    }


    public void determinePath()
    {
        if(chosenAllianceColor.getSelected() == Robot.constants.BLUE_ALLIANCE)
        {
            if(chosenPath.getSelected() == LEFT_POS)
            {
                Robot.auton.blueLeftFender();
            }
            else if(chosenPath.getSelected() == MIDDLE_POS_2_CARGO)
            {
                Robot.auton.blueMiddleTwoCargo();
            }
            else if(chosenPath.getSelected() == MIDDLE_POS_4_CARGO)
            {
                Robot.auton.blueMiddleFourCargo();
            }
            else if(chosenPath.getSelected() == RIGHT_POS)
            {
                Robot.auton.blueRightFender();
            }
        }
        else
        {
            if(chosenPath.getSelected() == LEFT_POS)
            {
                Robot.auton.redLeftFender();
            }
            else if(chosenPath.getSelected() == MIDDLE_POS_2_CARGO)
            {
                Robot.auton.redMiddleTwoCargo();
            }
            else if(chosenPath.getSelected() == MIDDLE_POS_4_CARGO)
            {
                Robot.auton.blueMiddleFourCargo();
            }
            else if(chosenPath.getSelected() == RIGHT_POS)
            {
                Robot.auton.redRightFender();
            }
        }
    }
}