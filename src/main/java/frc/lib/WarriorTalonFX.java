package frc.lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;


public class WarriorTalonFX extends TalonFX{

    //Slot 0 Config is used by WarriorTalonFX for PID and Feedforward
    private TalonFXConfiguration config;
    private MotorOutputConfigs motorConfig;
    private StrictFollower follow;
    private TrapezoidProfile motionProfile = null;
    Slot0Configs pid = new Slot0Configs();
    private CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withStatorCurrentLimit(40); 

    public WarriorTalonFX(int id, InvertedValue invert, NeutralModeValue brakeMode ){
        super(id);

        motorConfig = new MotorOutputConfigs().withNeutralMode(brakeMode).withInverted(invert);
        
        config = new TalonFXConfiguration().withMotorOutput(motorConfig).withCurrentLimits(currentConfig);
        
        this.getConfigurator().apply(config);

        String key = "Kraken " + this.getDeviceID() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
    public WarriorTalonFX(int id, InvertedValue invert, NeutralModeValue brakeMode, int currentLimit){
        super(id);

        motorConfig = new MotorOutputConfigs().withNeutralMode(brakeMode).withInverted(invert);

        currentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(currentLimit).withStatorCurrentLimit(currentLimit); 

        config = new TalonFXConfiguration().withMotorOutput(motorConfig).withCurrentLimits(currentConfig);
        
        this.getConfigurator().apply(config);

        String key = "Kraken " + this.getDeviceID() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
    
    public WarriorTalonFX(int id, InvertedValue invert, NeutralModeValue brakeMode, int currentLimit, TalonFX motorToFollow){
        super(id);

        motorConfig = new MotorOutputConfigs().withNeutralMode(brakeMode).withInverted(invert);

        currentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(currentLimit).withStatorCurrentLimit(currentLimit); 

        config = new TalonFXConfiguration().withMotorOutput(motorConfig).withCurrentLimits(currentConfig);
        
        follow = new StrictFollower(motorToFollow.getDeviceID());

        this.getConfigurator().apply(config);
        this.setControl(follow);
        String key = "Kraken " + this.getDeviceID() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
    


    //Following methods are UN-TESTED
    public void setPID(double P, double I, double D){
        pid.kP = P;
        pid.kI = I;
        pid.kD = D;
        this.getConfigurator().apply(pid);
    }

    public void setFeedForward(double kV){
        pid.kS = 0;
        pid.kV = kV;
        pid.kA = 0;
        pid.kG = 0;
        this.getConfigurator().apply(pid);
    }
    public void setFeedForward(double kV, double kA){
        pid.kS = 0;
        pid.kV = kV;
        pid.kA = kA;
        pid.kG = 0;
        this.getConfigurator().apply(pid);
    }
    public void setFeedForward(double kV, double kA, double kS, double kG){
        pid.kS = kS;
        pid.kV = kV;
        pid.kA = kA;
        pid.kG = kG;
        this.getConfigurator().apply(pid);
    }

    // this SETS a instance variable to store the motion profile
    public void setTrapezoidMotionProfile(double maxVel, double maxAccel){
         motionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }
    
    public TrapezoidProfile getTrapezoidProfile(){
        return motionProfile;
    }    

    //Uses Trap Profile if it is initalized, otherwise use PIDF
    public void goToPosition(double pos){
        TrapezoidProfile.State goal = new TrapezoidProfile.State(pos, 0);
        TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
        setpoint = motionProfile.calculate(0.020, setpoint, goal);

        PositionVoltage request = new PositionVoltage(0).withSlot(0);
        if(motionProfile != null){
            request.Position = setpoint.position;
            request.Velocity = setpoint.velocity;
            this.setControl(request);
        }else{
        this.setControl(request.withPosition(pos));
        }
    }
    //same thing but for velocity
    //e

    public void runVelocity(double RPS){
        TrapezoidProfile.State goal = new TrapezoidProfile.State(RPS, 0);
        TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

        setpoint = motionProfile.calculate(0.020, setpoint, goal);

        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        if(motionProfile != null){
            //this is so weird but its straight from the docs
            request.Velocity = setpoint.position;
            request.Acceleration = setpoint.velocity;
            this.setControl(request);
        }else{
            this.setControl(request.withVelocity(RPS));
        }
    }

}
