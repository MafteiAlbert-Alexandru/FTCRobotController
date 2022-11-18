package org.firstinspires.ftc.teamcode.hardware.customHardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ToggleButton {

    private final GamepadEx gamepadEx;
    private final GamepadKeys.Button button;
    public boolean toggle;

    public ToggleButton(GamepadEx gamepadEx, GamepadKeys.Button button){
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

