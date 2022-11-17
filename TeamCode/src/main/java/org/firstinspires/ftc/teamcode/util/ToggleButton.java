package org.firstinspires.ftc.teamcode.util;

public class ToggleButton {
    private int state = 0;

    private double currentSpeed;

    public void nextState() {
        if(state == 0) state = 1;
        else   state = 0;
    }

    public int getState() {
        return state;
    }
}