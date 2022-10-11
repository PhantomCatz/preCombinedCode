package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.DataLogger.*;
import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CatzClimber
{
    public boolean climbDataCollectionOn = false;
    
    public static WPI_TalonFX elevatorMtr;

    public static LimitSwitchNormal elevatorTopLimitSwitch;

    private final int ELEVATOR_MC_CAN_ID     = 50;
    public  final int ELEVATOR_MC_PDP_PORT   = 2;

    private final static double ELEVATOR_MTR_EXTEND_POWER  =  1.0;
    private final static double ELEVATOR_MTR_RETRACT_POWER = -0.5;
    private final static double ELEVATOR_MTR_STOP          =  0.0;
    
    private final int CURRENT_LIMIT_AMPS            = 60;
    private final int CURRENT_LIMIT_TRIGGER_AMPS    = 80;
    private final int CURRENT_LIMIT_TIMEOUT_SECONDS = 5;

    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private double elevatorCntsToInches = 0.0;
    private final double CNTS_TO_INCHES_CONVERSION_FACTOR = 27.0/216231.0;
    public static Timer elevatorMtrTimer;

    private double climbElevatorMtrPwr = 0.0;

    private SupplyCurrentLimitConfiguration climbCurrentLimit;


    //rotating arm  with pneumatics
    private DoubleSolenoid rotatingArmSolenoid;

    private final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;

    private final int ROTATING_ARM_DEPLOY_PCM_PORT = 5;
    private final int ROTATING_ARM_STOW_PCM_PORT   = 2;


    //arm extending motors ; ratio = 30:1
    public static CANSparkMax leftExtendingArmMtr;
    public static CANSparkMax rightExtendingArmMtr;

    private final int LT_EXT_ARM_MC_CAN_ID   = 53;
    private final int RT_EXT_ARM_MC_CAN_ID   = 52;

    public  final int LT_EXT_ARM_MC_PDP_PORT = 10;
    public  final int RT_EXT_ARM_MC_PDP_PORT = 6;

    private final double EXTEND_ARM_GEAR_RATIO = 30.0;

    private static RelativeEncoder extendArmEncoderRT;
    private static RelativeEncoder extendArmEncoderLT;

    private final int ROTATING_ARM_DEPLOYED = 1;
    private final int ROTATING_ARM_STOWED   = 0;
    private int deployRotatingArmState      = ROTATING_ARM_STOWED;

    private Thread elevatorThread;
    private final double CLIMB_THREAD_PERIOD = 0.02; //seconds

    private final static int CLIMB_STATE_IDLE                    = 0;
    private final static int CLIMB_STATE_ELEVATOR_MOVE_TO_POS    = 1;
    private final static int CLIMB_STATE_ELEVATOR_MOVE_TO_BOTTOM = 2;
    public static int climbState = CLIMB_STATE_IDLE;

    private double currentTime = 0.0;
    private boolean retractElevatorDone = false;
    private double targetDistanceInInches = 0.0;
    private double timeoutSeconds = 0.0;
    private double currentPosition = 0.0;

    public final double ELEVATOR_SPEED = 0.3;

    private final double INITIALIZE_TO_ZERO = 0.0;

    public CatzLog data;

    public final boolean MANUAL_MODE = true;
    
    public final boolean AUTONOMOUS_MODE = false;
    public boolean climbMode = MANUAL_MODE;

    private final double ELEVATOR_DETECT_BOTTOM_DURATION_SEC = 1; 
    private final int    ELEVATOR_DETECT_BOTTOM_DURATION_CNT = (int)(ELEVATOR_DETECT_BOTTOM_DURATION_SEC / CLIMB_THREAD_PERIOD);
    
    private static int elevatorDetectBottomCounter = 0;

    private final double ARMS_STARTING_ENCODER_COUNTS = 0.0;
    private double leftOuterArmPosition = 0.0;
    private double rightOuterArmPosition = 0.0;
    private final double OUTERARM_THRESHOLD = 0.0;
    private boolean firstTime = true;
    private double startingPosition = 0.0;

    public CatzClimber()
    {
        elevatorMtr = new WPI_TalonFX(ELEVATOR_MC_CAN_ID);
        elevatorMtr.configFactoryDefault();
        elevatorMtr.setNeutralMode(NeutralMode.Brake);

        //Set current limit
        climbCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        elevatorMtr.configSupplyCurrentLimit(climbCurrentLimit);

        elevatorMtr.set(TalonFXControlMode.Position, 0.0); 

        
        /**********************************************
        *   Rotating Arm with Pneumatics
        ***********************************************/
        rotatingArmSolenoid = new DoubleSolenoid(PCM_TYPE, ROTATING_ARM_DEPLOY_PCM_PORT, ROTATING_ARM_STOW_PCM_PORT);


        /**********************************************
        *   Outer Arm Extend/Retract
        ***********************************************/
        leftExtendingArmMtr = new CANSparkMax(LT_EXT_ARM_MC_CAN_ID, MotorType.kBrushless);
        leftExtendingArmMtr.restoreFactoryDefaults();
        leftExtendingArmMtr.setIdleMode(IdleMode.kBrake);

        rightExtendingArmMtr = new CANSparkMax(RT_EXT_ARM_MC_CAN_ID, MotorType.kBrushless);
        rightExtendingArmMtr.restoreFactoryDefaults();
        rightExtendingArmMtr.setIdleMode(IdleMode.kBrake);

        extendArmEncoderRT = rightExtendingArmMtr.getEncoder();
        extendArmEncoderRT.setPositionConversionFactor(360.0/EXTEND_ARM_GEAR_RATIO);
        extendArmEncoderRT.setPosition(ARMS_STARTING_ENCODER_COUNTS);

        extendArmEncoderLT = leftExtendingArmMtr.getEncoder();
        extendArmEncoderLT.setPositionConversionFactor(360.0/EXTEND_ARM_GEAR_RATIO);
        extendArmEncoderLT.setPosition(ARMS_STARTING_ENCODER_COUNTS);

        elevatorMtrTimer = new Timer();
        elevatorMtrTimer.reset();
        elevatorMtrTimer.start();

        climbProcessingThread();

        

    }

    public void climbProcessingThread()
    {

        elevatorThread = new Thread(() ->
        {
            while(true)
            {
                currentTime = elevatorMtrTimer.get();

                elevatorCntsToInches = (elevatorMtr.getSelectedSensorPosition(0) * CNTS_TO_INCHES_CONVERSION_FACTOR);
                currentPosition = elevatorCntsToInches;

                leftOuterArmPosition = extendArmEncoderLT.getPosition();
                rightOuterArmPosition = extendArmEncoderRT.getPosition();


                if(climbMode == AUTONOMOUS_MODE)
                {
                    switch(climbState)
                    {
                        
                        case CLIMB_STATE_IDLE:
                            elevatorMtrPwrOff();                       
                        break; 


                        case CLIMB_STATE_ELEVATOR_MOVE_TO_POS:
                            if (retractElevatorDone == false)
                            {

                                if (targetDistanceInInches < 0.0)
                                {
                                    elevatorMtr.set(TalonFXControlMode.PercentOutput, ELEVATOR_MTR_RETRACT_POWER);

                                    if (currentPosition >= targetDistanceInInches)
                                    {
                                        elevatorMtr.set(TalonFXControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                                        retractElevatorDone = true;
                                    }              
                                } 
                                else if(targetDistanceInInches > 0.0) 
                                {
                                    elevatorMtr.set(TalonFXControlMode.PercentOutput, ELEVATOR_MTR_EXTEND_POWER);

                                    if (currentPosition >= targetDistanceInInches)
                                    {
                                        elevatorMtr.set(TalonFXControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                                        retractElevatorDone = true;
                                    }
                                }

                                if(currentTime > timeoutSeconds)
                                {
                                    elevatorMtr.set(TalonFXControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                                    retractElevatorDone = true; 
                                    climbState = CLIMB_STATE_IDLE;

                                }
                            }
                        break;

                        case CLIMB_STATE_ELEVATOR_MOVE_TO_BOTTOM:

                                if(elevatorDetectBottomCounter == 0)
                                {
                                    elevatorMtr.set(TalonFXControlMode.PercentOutput, ELEVATOR_SPEED);
                                }

                                if(elevatorDetectBottomCounter > ELEVATOR_DETECT_BOTTOM_DURATION_CNT)
                                {
                                    climbState = CLIMB_STATE_IDLE;
                                    elevatorMtr.setSelectedSensorPosition(INITIALIZE_TO_ZERO);
                                }
                                elevatorDetectBottomCounter++;
                            
                        break;
                        
                        default:
                        break;   
                        
                                
                    }
                }

                //checkOuterArmPos();

                if(climbDataCollectionOn == true)
                {
                    if(Robot.xboxAux.getRightBumper())
                    {
                        //Only log data while we are climbing
                        data = new CatzLog(currentTime, climbElevatorMtrPwr, elevatorMtr.getMotorOutputPercent(),
                                                                            elevatorMtr.getMotorOutputVoltage(), 
                                                                            elevatorMtr.getSupplyCurrent(),
                                                                            elevatorMtr.getTemperature(),
                                                                            currentPosition,

                                                                            rightExtendingArmMtr.getAppliedOutput(),
                                                                            rightExtendingArmMtr.getOutputCurrent(),
                                                                            rightExtendingArmMtr.getMotorTemperature(),
                                                                            rightOuterArmPosition,
                                                                            
                                                                            leftExtendingArmMtr.getAppliedOutput(),
                                                                            leftExtendingArmMtr.getOutputCurrent(),
                                                                            leftExtendingArmMtr.getMotorTemperature(), 
                                                                            leftOuterArmPosition,

                                                                            deployRotatingArmState);  
                        Robot.dataCollection.logData.add(data);   
                    }
                }
                

                
                Timer.delay(CLIMB_THREAD_PERIOD); 
            }
        });
        elevatorThread.start();
    }


    /*----------------------------------------------------------------------------------------------
    * 
    *  Elevator functions 
    *
    *---------------------------------------------------------------------------------------------*/
    public void climbElevatorManualCntrl(double motorPower)
    {

        climbElevatorMtrPwr = motorPower;
        elevatorMtr.set(climbElevatorMtrPwr);
        /*if(elevatorCntsToInches <= 1.0)
        {
            if(climbElevatorMtrPwr > 0.0)
            {
                elevatorMtr.set(climbElevatorMtrPwr);
            }
            else
            {
                elevatorMtr.set(0.0);
            }
        }
        else
        {
            elevatorMtr.set(climbElevatorMtrPwr);
        }*/
        
        
    }

    public void elevatorMtrPwrOff()
    {
        climbElevatorMtrPwr = ELEVATOR_MTR_STOP;
        
        elevatorMtr.set(climbElevatorMtrPwr);
    }

    public void moveElevatorToPosition(double distanceInInches, double timeoutSeconds)
    {
        climbState = CLIMB_STATE_ELEVATOR_MOVE_TO_POS;
        
    }

    public static void elevatorDetectBottom()
    {
        elevatorDetectBottomCounter = 0;
        climbState = CLIMB_STATE_ELEVATOR_MOVE_TO_BOTTOM;
    }


     /*----------------------------------------------------------------------------------------------
    * 
    *  Outer Arm Deployment functions 
    *
    *---------------------------------------------------------------------------------------------*/
    public void climbRotatingArmDeploy()
    {
        rotatingArmSolenoid.set(Value.kForward);
        deployRotatingArmState = ROTATING_ARM_DEPLOYED;
    }

    public void climbRotatingArmStow()
    {
        rotatingArmSolenoid.set(Value.kReverse);
        deployRotatingArmState = ROTATING_ARM_STOWED;
    }


    /*----------------------------------------------------------------------------------------------
    * 
    *  Outer Arm Extend/Retract Functions
    *
    *---------------------------------------------------------------------------------------------*/
    public void climbExtendRotatingArmManualCtrlBOTH(double motorPower)
    {
        double extendingArmScaleFactor = 1.0;//1.00;
        
        if(firstTime == true)
        {
            startingPosition = leftOuterArmPosition;
            firstTime = false;
        }
        
        // Right arm is faster than left, so modify right arm speeds
        // Right arm speed is different going down vs up
        /*if(motorPower < 0)
        {
            extendingArmScaleFactor = 0.9; //start at 95
        }
        else
        {
            extendingArmScaleFactor = 0.9;
        }*/

        rightExtendingArmMtr.set(motorPower);
        leftExtendingArmMtr.set(-motorPower * extendingArmScaleFactor);
    }

    public void climbExtendRotatingArmManualCtrlRT(double motorPower)
    {
        rightExtendingArmMtr.set(motorPower);
    }

    public void climbExtendRotatingArmManualCtrlLT(double motorPower)
    {
        leftExtendingArmMtr.set(-motorPower);
    }


    /*----------------------------------------------------------------------------------------------
    * 
    *  MISC functions
    *
    *---------------------------------------------------------------------------------------------*/

    public void checkOuterArmPos()
    {
        double deltaPos = leftOuterArmPosition - startingPosition;
        if(deltaPos > OUTERARM_THRESHOLD)
        {
            Robot.climbLimitReached = true;
        }
    }
    public void setToCoastMode()
    {
        elevatorMtr.setNeutralMode(NeutralMode.Coast);
    }

    /*----------------------------------------------------------------------------------------------
    * 
    *  Smart Dashboard
    *
    *---------------------------------------------------------------------------------------------*/
    public void smartDashboardClimb()
    {
        SmartDashboard.putNumber("ElevatorMtrTemp", elevatorMtr.getTemperature());
    }

    public void smartDashboardClimb_DEBUG()
    {
        SmartDashboard.putNumber("Left Arm Position", leftOuterArmPosition);
       // SmartDashboard.putNumber("Left Arm Temp", leftExtendingArmMtr.getMotorTemperature());

        SmartDashboard.putNumber("Right Arm Position", rightOuterArmPosition);
        //SmartDashboard.putNumber("Right Arm Temp", rightExtendingArmMtr.getMotorTemperature());
        /*
        SmartDashboard.putNumber("encoder value", elevatorMtr.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Elevator Position", targetDistanceInInches);
        SmartDashboard.putNumber("position", currentPosition);*/
    }

}