package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ToggleButton {

    private final GamepadKeys.Button button;
    public boolean toggle;

    public ToggleButton( GamepadKeys.Button button){
        this.button = button;

    }

    public void update(GamepadEx gamepadEx){
        if(gamepadEx.wasJustPressed(button)) {
            toggle = !toggle;
        }
    }

    public boolean getToggle(){
        return toggle;
    }
}

