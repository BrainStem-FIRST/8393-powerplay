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
    private Telemetry telemetry;

    public ServoImplEx leftFlipper;
    public ServoImplEx rightFlipper;


    public final String SYSTEM_NAME = "FLIPPERS";
    public final String LEFT_DOWN = "LEFT_DOWN";
    public final String RIGHT_DOWN = "RIGHT_DOWN";
    public final String LEFT_UP = "LEFT_UP";
    public final String RIGHT_UP = "RIGHT_UP";
    public final String BOTH_DOWN = "BOTH_DOWN";
    public final String BOTH_UP = "BOTH_UP";


    public final double LEFT_DOWN_POSITION = 10; //FIXME
    public final double RIGHT_DOWN_POSITION = 10; //FIXME
    public final double LEFT_UP_POSITION = 10; //FIXME
    public final double RIGHT_UP_POSITION = 10; //FIXME


    private Map stateMap;

    private boolean isAuto;

    public Flippers(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;


        leftFlipper = new CachingServo(hwMap.get(ServoImplEx.class, "leftFlipper"));
        rightFlipper = new CachingServo(hwMap.get(ServoImplEx.class, "rightFlipper"));
        leftFlipper.setPwmRange(new PwmControl.PwmRange(LEFT_UP_POSITION, LEFT_DOWN_POSITION));
        rightFlipper.setPwmRange(new PwmControl.PwmRange(RIGHT_UP_POSITION, RIGHT_DOWN_POSITION));

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

    public void setState() {
        switch(SYSTEM_NAME){
            case LEFT_DOWN: {
                leftFlipper.setPosition(1.0);
                break;
            }
            case RIGHT_DOWN: {
                rightFlipper.setPosition(1.0);
                break;
            }
            case LEFT_UP: {
                leftFlipper.setPosition(0);
                break;
            }
            case RIGHT_UP: {
                rightFlipper.setPosition(0.0);
                break;
            }
            case BOTH_DOWN: {
                rightFlipper.setPosition(1.0);
                leftFlipper.setPosition(1.0);
                break;
            }
            case BOTH_UP: {
                rightFlipper.setPosition(0.0);
                leftFlipper.setPosition(0.0);
            }
        }
    }
}
