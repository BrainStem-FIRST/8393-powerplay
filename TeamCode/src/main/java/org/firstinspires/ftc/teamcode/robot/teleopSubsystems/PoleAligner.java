package org.firstinspires.ftc.teamcode.robot.teleopSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import java.util.Map;


public class PoleAligner implements Subsystem {

    private static final class PoleAlignerConstants {
        private static final int UP_POSITION = 0; //FIXME
        private static final int DOWN_POSITION = 2250; //FIXME
    }

    private Telemetry telemetry;

    public ServoImplEx poleAlignerServo;


    public final String SYSTEM_NAME = "POLE_ALIGNER";
    public final String DOWN = "DOWN";
    public final String UP = "UP";


    private Map stateMap;

    private boolean isAuto;

    public PoleAligner(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {

        this.telemetry = telemetry;
        this.stateMap = stateMap;


        poleAlignerServo = new CachingServo(hwMap.get(ServoImplEx.class, "rightTurretServo"));
        poleAlignerServo.setPwmRange(new PwmControl.PwmRange(PoleAlignerConstants.UP_POSITION, PoleAlignerConstants.DOWN_POSITION));

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

    public void setState(String desiredState) {
        switch(desiredState){
            case UP: {
                poleAlignerServo.setPosition(0.73);
                break;
            }
            case DOWN: {
                poleAlignerServo.setPosition(0.28);
                break;
            }
        }
    }
}