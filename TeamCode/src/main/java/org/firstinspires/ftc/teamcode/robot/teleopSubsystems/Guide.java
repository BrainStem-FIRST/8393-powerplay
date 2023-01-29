package org.firstinspires.ftc.teamcode.robot.teleopSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import java.util.Map;

public class Guide implements Subsystem {
    public final String     SYSTEM_NAME = "GUIDE";
    public final String     UP_POSITION = "UP_STATE";
    public final String     DOWN_POSITION = "DOWN_STATE";
    public final double     LEFT_POSITION_SERVO_VALUE = 780;
    public final double     RIGHT_POSITION_SERVO_VALUE = 1950;

    public Telemetry telemetry;
    private ServoImplEx guideServo;
    private Map stateMap;

    public Guide(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        guideServo = new CachingServo(hwMap.get(ServoImplEx.class, "guide"));

        guideServo.setPwmRange(new PwmControl.PwmRange(LEFT_POSITION_SERVO_VALUE,  RIGHT_POSITION_SERVO_VALUE));
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
        selectTransition((String) stateMap.get(SYSTEM_NAME));
    }

    public void selectTransition(String desiredPosition){
        switch(desiredPosition){
            case UP_POSITION:{
                transitionToPosition(0.99);
                break;
            } case DOWN_POSITION:{
                transitionToPosition(0.01);
                break;
            }
        }
    }

    public void transitionToPosition (double position) {
        //raising heights to reach different junctions, so four values
        guideServo.setPosition(position);
    }
}
