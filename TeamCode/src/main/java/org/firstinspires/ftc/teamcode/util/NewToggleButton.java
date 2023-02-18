package org.firstinspires.ftc.teamcode.util;

public class NewToggleButton {
    private boolean previousState = false, currentState = false;
    private boolean toggle = false;

    public NewToggleButton() {
        toggle = false;
    }

    public NewToggleButton(boolean initalToggleState)
    {
        toggle = initalToggleState;
    }

    public boolean getState()
    {
        return toggle;
    }

    public void setState(boolean state) {
        toggle = state;
        previousState = false;
        currentState = false;
    }

    public boolean update(boolean currentState) {
        previousState = this.currentState;
        this.currentState = currentState;

        if (currentState && currentState != previousState)
            toggle = !toggle;

        return toggle;
    }
}