package org.firstinspires.ftc.teamcode.robot.teleopSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;


public class Grabber implements Subsystem {
    private Telemetry telemetry;

    public ServoImplEx grabber;


    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String FULLY_OPEN = "FULLYOPEN";
    public final String CLOSED_STATE = "CLOSED";
    public final String CLOSED_STATE_CAP = "CLOSED_CAP";
    Constants constants = new Constants();

    public final double REGULAR_OPEN = 1350;
    public final double CLOSED_VALUE = 2250;

    public boolean cap = false;

    private Map stateMap;

    private boolean isAuto;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap, boolean isAuto) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.isAuto = isAuto;


        grabber = new CachingServo(hwMap.get(ServoImplEx.class, "grabber"));

        grabber.setPwmRange(new PwmControl.PwmRange(REGULAR_OPEN, CLOSED_VALUE));
        stateMap.put(constants.GRABBER_MODE, constants.DEPOSITING);

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

    public void setState(Lift lift) {
        if (stateMap.get(SYSTEM_NAME) == CLOSED_STATE) {
            if (cap) {
                opencap();
            } else {
                close();
            }
        } else if  (stateMap.get(SYSTEM_NAME) == CLOSED_STATE_CAP) {

        } else if (stateMap.get(SYSTEM_NAME) == OPEN_STATE) {
            open();
        }  else if (stateMap.get(SYSTEM_NAME) == FULLY_OPEN) {
            if (lift.getAvgLiftPosition() > 350) {
                maxOpen();
            } else {
                open();
            }
        }
    }

    public void open() {
        grabber.setPosition(0.52);
    }

    public void opencap() { grabber.setPosition(0.875);}

    public void maxOpen() {
        grabber.setPosition(0.0);
    }

    public void close() {
        grabber.setPosition(0.99);
    }
}
