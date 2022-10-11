package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CatzIntake
{
    public WPI_TalonSRX intakeRoller;

    private final int TALON_MC_ID            = 30; 
    private final int INTAKE_DEPLOY_PCM_PORT  = 6;
    private final int INTAKE_RETRACT_PCM_PORT = 7;

    private final double INTAKE_ROLLER_MOTOR_POWER      = 0.85;//0.78;
    private final double OUTTAKE_ROLLER_MOTOR_POWER     = 1.0;
    public final double AUTON_INTAKE_ROLLER_MOTOR_POWER = 0.8;
    private final double INTAKE_MOTOR_POWER_OFF         = 0.0;

    public final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
    private DoubleSolenoid intakeDeploySolenoid;

    public boolean intakeRollerOn = false;
    public boolean intakeRollerOut = false;

    private SupplyCurrentLimitConfiguration intakeCurrentLimit;

    private final int        CURRENT_LIMIT_AMPS            = 50;
    private final int        CURRENT_LIMIT_TRIGGER_AMPS    = 50;
    private final double     CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean    ENABLE_CURRENT_LIMIT          = true;

    public boolean intakeStowed = false;

    public CatzIntake()
    {
        intakeRoller         = new WPI_TalonSRX(TALON_MC_ID);
        intakeDeploySolenoid = new DoubleSolenoid(PCM_TYPE, INTAKE_DEPLOY_PCM_PORT, INTAKE_RETRACT_PCM_PORT); 
        intakeRoller.configFactoryDefault();
        intakeCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
        intakeRoller.configSupplyCurrentLimit(intakeCurrentLimit);
        
        intakeRoller.setStatusFramePeriod(1, 255);
        intakeRoller.setStatusFramePeriod(2, 255);
        stowIntake();
    }
        
//-----------------------------------------------------------Roller----------------------------------------------------------

    public void intakeRollerIn()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_MOTOR_POWER);
        intakeRollerOn = true;
    }

    public void intakeRollerInAuton()
    {
        intakeRoller.set(ControlMode.PercentOutput, AUTON_INTAKE_ROLLER_MOTOR_POWER);
        intakeRollerOn = true;
    }

    public void intakeRollerOut()
    {
        intakeRoller.set(ControlMode.PercentOutput, -OUTTAKE_ROLLER_MOTOR_POWER);
        intakeRollerOut = true;
    }

    public void intakeRollerOff()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);
        intakeRollerOn = false;
        intakeRollerOut = false;
    }

//-----------------------------------------------------------Deploy/Stow--------------------------------------------------------

    public void deployIntake() 
    {
        intakeDeploySolenoid.set(Value.kForward);
        intakeStowed = false;
    }

    public void stowIntake() 
    {
        intakeDeploySolenoid.set(Value.kReverse);
        intakeStowed = true;
    }

    public Value getIntakeState()
    {
        return intakeDeploySolenoid.get();
    }
}