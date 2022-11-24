package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;



public class Grabber {
    private Telemetry telemetry;

    public ServoImplEx grabber;


    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String FULLY_OPEN = "FULLYOPEN";
    public final String CLOSED_STATE = "CLOSED";
    Constants constants = new Constants();



    public final double MAX_OPEN = 900;
    public final double REGULAR_OPEN = 1375;
    public final double CLOSED_VALUE = 1850;

    private Map stateMap;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;


        grabber = new CachingServo(hwMap.get(ServoImplEx.class, "grabber"));

        grabber.setPwmRange(new PwmControl.PwmRange(MAX_OPEN, CLOSED_VALUE));

    }

    public void setState(Lift lift) {
        if(((String)stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
            if (shouldGrab(lift)) {
                grabber.setPosition(1);
            } else {
                grabber.setPosition(0);
            }

            if (stateMap.get(constants.GRABBER_START_TIME) == null) {
                stateMap.put(constants.GRABBER_START_TIME, System.currentTimeMillis());
            } else {
                long grabberStartTime = (long) stateMap.get(constants.GRABBER_START_TIME);
                long grabberEndTime = grabberStartTime + constants.GRABBER_CYCLE_TIME;
                if(System.currentTimeMillis() > grabberEndTime) {
                    stateMap.put(constants.GRABBER_START_TIME, null);
                    stateMap.put(constants.CYCLE_GRABBER, constants.STATE_COMPLETE);
                }
            }

        } else if (((String)stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_NOT_STARTED) && shouldGrab(lift)) {
            grabber.setPosition(0);
        }

        telemetry.addData("grabberPosition", grabber.getPosition());
    }

    public void open() {
        grabber.setPosition(REGULAR_OPEN/CLOSED_VALUE);
    }

    public void maxOpen(){grabber.setPosition(0);}

    public boolean shouldGrab(Lift lift) {
        return lift.getPosition() < lift.LIFT_POSITION_GROUND &&
                ((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }

    public void actuallySettingGrabberState(String state){
        stateMap.put(SYSTEM_NAME, state);
    }

    public void runGrabber(String grabberState){
        if(stateMap.get(SYSTEM_NAME) == grabberState){
            return;
        }
        if(grabberState == this.OPEN_STATE){
            grabber.setPosition(1);
        } else {
            grabber.setPosition(0);
        }
    }
}