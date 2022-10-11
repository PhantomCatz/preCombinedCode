package frc.Mechanisms;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.Robot;

public class CatzRGB 
{
    private final int LED_COUNT = 22;
    private final int COLOR1_LED_COUNT = LED_COUNT / 2;

    private final int MAX_LED_BUFFER_INDEX = LED_COUNT - 1;

    private final int LED_PWM_PORT = 0;

    private AddressableLED led;

    private AddressableLEDBuffer ledBuffer;
    
    private Color RED    = Color.kRed;
    private Color ORANGE = Color.kOrange;
    private Color YELLOW = Color.kYellow;
    private Color GREEN  = Color.kGreen;
    private Color BLUE   = Color.kBlue;
    private Color PURPLE = Color.kPurple;
    private Color WHITE  = Color.kWhite;
    private Color TEAM_COLOR = Color.kDodgerBlue;
    private Color PINK  = Color.kHotPink;

    private Color color1;
    private Color color2;

    private int ledIndex = 0;
    private int flowArrayOffset = 0;
    private int arrayIndex = 0;

    private int ledDelay = 0;
    private final int FLOW_UP_DELAY_COUNT = 2;
    private final int FLOW_DN_DELAY_COUNT = 2;
    private final int FLASH_DELAY_COUNT   = 15;

    private int nextFlashColor = 1;

    private ArrayList<Color> ledPattern;


    public CatzRGB()
    {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        ledPattern = new ArrayList<Color>(LED_COUNT);

        for(int i = 0; i < LED_COUNT; i++)
        {
            ledPattern.add(i, TEAM_COLOR);
        }
    }

    public void setLEDPattern(Color color1, Color color2)
    {
        for(int i = 0; i < COLOR1_LED_COUNT; i++)
        {
            ledPattern.set(i, color1);
        }
        for(int i = COLOR1_LED_COUNT; i < LED_COUNT; i++)
        {
            ledPattern.set(i, color2);
        }
    }

    public void setLEDPattern(Color color)
    {
        for(int i = 0; i < LED_COUNT; i++)
        {
            ledPattern.set(i, color);
        }
    }

    public void LEDWork()
    {
        if(Robot.robotDisabled)
        {
            solidColor(TEAM_COLOR);
        }
        else if(Robot.isClimbing == true)
        {
            flash(BLUE, WHITE);
            if(Robot.climbLimitReached == true)
            {
                flash(PINK,WHITE);
            }
        }
        else if(Robot.defenseMode == true)
        {
            flash(ORANGE,WHITE);
        }
        else if(Robot.forwardSwitchHit == true)
        {
            flash(GREEN,WHITE);
        }
        else if(Robot.reverseSwitchHit == true)
        {
            flash(RED, WHITE);
        }
        else
        {
            if(Robot.targetAquired == true)
            {
                color1 = GREEN;
            }
            else
            {
                color1 = PURPLE;
            }

            if(Robot.inShootingRange == true)
            {
                color2 = BLUE;
            }
            else
            {
                color2 = YELLOW;
            }

            setLEDPattern(color1, color2);

            if(ledDelay > FLOW_UP_DELAY_COUNT)
            {
                flowUp();
                ledDelay = 0;
            }
    
        }
        ledDelay++;
        led.setData(ledBuffer);
    }
    
    public void flash(Color color1, Color color2)
    {
        if(ledDelay > FLASH_DELAY_COUNT)
        {
            if(nextFlashColor == 1)
            {
                solidColor(color1);
                nextFlashColor = 2;
            }
            else
            {
                solidColor(color2);
                nextFlashColor = 1;
            }

            ledDelay = 0;
        }
    }

    public void solidColor(Color color)
    {
        for(int i = 0; i < LED_COUNT; i++)
        {
            ledBuffer.setLED(i, color);
        }
    }

    public void flowDown()
    {
        for(ledIndex = 0; ledIndex < LED_COUNT; ledIndex++)
        {
            arrayIndex = (flowArrayOffset + ledIndex) % LED_COUNT;
            ledBuffer.setLED(ledIndex, ledPattern.get(arrayIndex));
        }
        
        flowArrayOffset++;

        if(flowArrayOffset >= LED_COUNT)
        {
            flowArrayOffset = 0;
        }
    }

    public void flowUp()
    {
        for(ledIndex = 0; ledIndex < LED_COUNT; ledIndex++)
        {
            arrayIndex = (flowArrayOffset + ledIndex) % LED_COUNT;
            ledBuffer.setLED(ledIndex, ledPattern.get(arrayIndex));
        }
        
        flowArrayOffset--;

        if(flowArrayOffset < 0)
        {
            flowArrayOffset = MAX_LED_BUFFER_INDEX;
        }
    }
}
