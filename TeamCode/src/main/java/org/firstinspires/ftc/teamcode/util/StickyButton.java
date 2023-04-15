package org.firstinspires.ftc.teamcode.util;

public class StickyButton {
    private boolean previousState = false, currentState = false;

    public boolean getState() { return currentState && currentState != previousState;}

    public boolean getOppositeState() { return !currentState && currentState != previousState; }

    public void update(boolean currentState) {
        previousState = this.currentState;
        this.currentState = currentState;
    }
}