package org.firstinspires.ftc.teamcode.robot.teleopSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import java.util.Map;


public class Flippers implements Subsystem {

    private static final class FlippersConstants {
        private static final int LEFT_DOWN_POSITION = 2250; //FIXME
        private static final int RIGHT_DOWN_POSITION = 0; //FIXME
        private static final int LEFT_UP_POSITION = 0; //FIXME
        private static final int RIGHT_UP_POSITION = 2250; //FIXME

    }
    private Telemetry telemetry;

    public ServoImplEx leftFlipper;
    public ServoImplEx rightFlipper;


    public final String RIGHT_SYSTEM_NAME = "RIGHT_FLIPPERS";
    public final String LEFT_SYSTEM_NAME = "LEFT_FLIPPERS";
    public final String LEFT_DOWN = "LEFT_DOWN";
    public final String RIGHT_DOWN = "RIGHT_DOWN";
    public final String LEFT_UP = "LEFT_UP";
    public final String RIGHT_UP = "RIGHT_UP";





    private Map stateMap;

    private boolean isAuto;

    public Flippers(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {

        this.telemetry = telemetry;
        this.stateMap = stateMap;


        leftFlipper = new CachingServo(hwMap.get(ServoImplEx.class, "leftFlipper"));
        rightFlipper = new CachingServo(hwMap.get(ServoImplEx.class, "rightFlipper"));
        leftFlipper.setPwmRange(new PwmControl.PwmRange(FlippersConstants.LEFT_UP_POSITION, FlippersConstants.LEFT_DOWN_POSITION));
        rightFlipper.setPwmRange(new PwmControl.PwmRange(FlippersConstants.RIGHT_UP_POSITION, FlippersConstants.RIGHT_DOWN_POSITION));

    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    public void setState(String leftDesiredState, String rightDesiredState) {
        switch(leftDesiredState){
            case LEFT_DOWN: {
                leftFlipper.setPosition(0.73);
                break;
            }
            case LEFT_UP: {
                leftFlipper.setPosition(0.28);
                break;
            }
        }

        switch(rightDesiredState){
            case RIGHT_DOWN: {
                rightFlipper.setPosition(0.735);
                break;
            }
            case RIGHT_UP: {
                rightFlipper.setPosition(0.28);
                break;
            }
        }
    }
}