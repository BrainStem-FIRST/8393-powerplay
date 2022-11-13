package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;


import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;



public class Grabber {
    private Telemetry telemetry;
    private ServoImplEx grabber;


    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String CLOSED_STATE = "CLOSED";
    Constants constants = new Constants();


    public final double OPEN_VALUE = 1400; //was 1600
    public final double CLOSED_VALUE = 2000; //was 1125

    private Map stateMap;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;


        grabber = new CachingServo(hwMap.get(ServoImplEx.class, "grabber"));

        grabber.setPwmRange(new PwmControl.PwmRange(CLOSED_VALUE, OPEN_VALUE));
    }

    /*public void setState(Lift lift) {
        if (shouldGrab(lift)) {
            grabber.setPosition(CLOSED_VALUE);
        } else {
            grabber.setPosition(OPEN_VALUE);
        }

        if(((String)stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
            if (shouldGrab(lift)) {
                grabber.setPosition(0);
            } else {
                grabber.setPosition(1);
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
            grabber.setPosition(1);
        }

        telemetry.addData("grabberPosition", grabber.getPosition());
    }*/

    public void setState(){
        if (stateMap.get(SYSTEM_NAME).equals(OPEN_STATE)) {
            grabber.setPosition(1);
        } else if (stateMap.get(SYSTEM_NAME).equals(CLOSED_STATE)) {
            grabber.setPosition(0);
        }
    }
    public void openGrabber() {
        grabber.setPosition(1);
    }

    public void closeGrabber() {
        grabber.setPosition(0);
    }

    public boolean shouldGrab(Lift lift) {
        return lift.getPosition() < lift.LIFT_HEIGHT_GROUND &&
                ((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }
}