package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class LockableLoggedNetworkBoolean extends LoggedNetworkBoolean{
    public boolean isLocked = false;
    public boolean lockedValue = false;
    public LockableLoggedNetworkBoolean(String key, boolean defaultValue){
        super(key,defaultValue);
    }
    public void lockBoolean(){
        lockedValue = super.getAsBoolean();
        isLocked = true;
    }

    @Override
    public boolean getAsBoolean() {
        if(isLocked){
            return lockedValue;
        }
        return super.getAsBoolean();
    }
}
