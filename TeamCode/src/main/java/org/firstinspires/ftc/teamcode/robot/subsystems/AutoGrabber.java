package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import java.util.Map;


public class AutoGrabber {
    private Telemetry telemetry;

    public ServoImplEx grabber;


    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String FULLY_OPEN = "FULLYOPEN";
    public final String CLOSED_STATE = "CLOSED";
    Constants constants = new Constants();



    public final double DONT_USE_THIS = 900;
    public final double REGULAR_OPEN = 1425;
    public final double CLOSED_VALUE = 1850;

    private Map stateMap;

    public AutoGrabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;


        grabber = new CachingServo(hwMap.get(ServoImplEx.class, "grabber"));

        grabber.setPwmRange(new PwmControl.PwmRange(REGULAR_OPEN, CLOSED_VALUE));
        stateMap.put(constants.GRABBER_MODE, constants.DEPOSITING);

    }

    public void setState(AutoLift lift) {
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

//        telemetry.addData("grabberPosition", grabber.getPosition());
    }

    public void open() {
        grabber.setPosition(0);
    }

    public void maxOpen(){grabber.setPosition(0);}

    public void close(){
        grabber.setPosition(1);
//        telemetry.addData("grabber closing", grabber.getPosition());
    }

    public boolean shouldGrab(AutoLift lift) {
        return (lift.getPosition() < lift.LIFT_POSITION_GROUND) &&
                ((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }
}