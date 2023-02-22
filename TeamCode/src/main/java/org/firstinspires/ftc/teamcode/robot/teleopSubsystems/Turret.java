package org.firstinspires.ftc.teamcode.robot.teleopSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;

public class Turret implements Subsystem {
    public final String     SYSTEM_NAME = "TURRET";
    public final String     LEFT_POSITION = "LEFT_STATE";
    public final String     RIGHT_POSITION = "RIGHT_STATE";
    public final String     CENTER_POSITION = "CENTER_STATE";
    public final String     RIGHT_POSITION_AUTO = "RIGHT_POSITION_AUTO";
    public final double     LEFT_POSITION_SERVO_VALUE = 532;
    public final double     CENTER_POSITION_SERVO_VALUE = 1375;
    public final double     RIGHT_POSITION_SERVO_VALUE = 2166;
    public final int        LIFT_MIN_HEIGHT_TO_MOVE_TURRET = 309;
    public final double TURRET_CENTER_POSITION = 0.51;

    public Telemetry telemetry;
    private ServoImplEx leftTurretServo;
    //private ServoImplEx rightTurretServo;
    private Map stateMap;
    private boolean isAuto;

    public Turret(HardwareMap hwMap, Telemetry telemetry, Map stateMap, boolean isAuto) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.isAuto = isAuto;

        leftTurretServo = new CachingServo(hwMap.get(ServoImplEx.class, "turretLeft"));
        //rightTurretServo = new CachingServo(hwMap.get(ServoImplEx.class, "turretRight"));

        leftTurretServo.setPwmRange(new PwmControl.PwmRange(LEFT_POSITION_SERVO_VALUE,  RIGHT_POSITION_SERVO_VALUE));
        //rightTurretServo.setPwmRange(new PwmControl.PwmRange(LEFT_POSITION_SERVO_VALUE,  RIGHT_POSITION_SERVO_VALUE));
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

    public void setState(Lift lift){
        if(isLiftTooLow(lift)){
            selectTransition((String) CENTER_POSITION);
        }
        else{
            selectTransition((String) stateMap.get(SYSTEM_NAME));
        }
    }

    public boolean isLiftTooLow(Lift lift) {
        if(isAuto){
            return false;
        } else {
            if(stateMap.get(lift.LIFT_SYSTEM_NAME).equals(lift.LIFT_POLE_LOW)){
                return lift.getPosition() < 235;
            } else {
                return lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_TURRET;
            }
        }
    }

    public void selectTransition(String desiredLevel){
        switch(desiredLevel){
            case LEFT_POSITION:{
                transitionToPosition(0.03);
                break;
            } case CENTER_POSITION:{
                transitionToPosition(TURRET_CENTER_POSITION);
                break;
            } case RIGHT_POSITION:{
                transitionToPosition(1);
                break;
            }
            case RIGHT_POSITION_AUTO: {
                transitionToPosition(0.9);
            }
        }
    }

    public void transitionToPosition (double position) {
        //raising heights to reach different junctions, so four values
        leftTurretServo.setPosition(position);
        //rightTurretServo.setPosition(position);
    }

    public void centerTurret(){
        transitionToPosition(TURRET_CENTER_POSITION);
    }
}
