package org.firstinspires.ftc.teamcode.hardware.customHardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class SmartToggle {

    private GamepadEx gamepadEx;
    private GamepadKeys.Button button;
    private boolean bool;
    public boolean toggle;

    public SmartToggle(GamepadEx gamepadEx, GamepadKeys.Button button){
        this.gamepadEx = gamepadEx;
        this.button = button;
    }

    public void update(){
        if(gamepadEx.wasJustPressed(button)) {
            toggle = !toggle;
        }
    }

    public boolean getToggle(){
        return toggle;
    }
}
