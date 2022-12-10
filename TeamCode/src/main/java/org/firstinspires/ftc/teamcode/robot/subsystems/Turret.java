package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;

public class Turret {
    public final String     SYSTEM_NAME = "TURRET";
    public final String     LEFT_POSITION = "LEFT_STATE";
    public final String     RIGHT_POSITION = "RIGHT_STATE";
    public final String     CENTER_POSITION = "CENTER_STATE";
    public final double     LEFT_POSITION_LEFT_SERVO_VALUE = 549;
    public final double     LEFT_POSITION_RIGHT_SERVO_VALUE = 549;
    public final double     CENTER_POSITION_LEFT_SERVO_VALUE = 1373;
    public final double     CENTER_POSITION_RIGHT_SERVO_VALUE = 1373;
    public final double     RIGHT_POSITION_LEFT_SERVO_VALUE = 2155;
    public final double     RIGHT_POSITION_RIGHT_SERVO_VALUE = 2155;
    public final int        LIFT_MIN_HEIGHT_TO_MOVE_TURRET = 309;
    public final double TURRET_CENTER_POSITION = 0.52;

    public Telemetry telemetry;
    private ServoImplEx leftTurretServo;
    private ServoImplEx rightTurretServo;
    private Map stateMap;
    private boolean isAuto;

    public Turret(HardwareMap hwMap, Telemetry telemetry, Map stateMap, boolean isAuto) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.isAuto = isAuto;

        leftTurretServo = new CachingServo(hwMap.get(ServoImplEx.class, "turretLeft"));
        rightTurretServo = new CachingServo(hwMap.get(ServoImplEx.class, "turretRight"));

        leftTurretServo.setPwmRange(new PwmControl.PwmRange(LEFT_POSITION_LEFT_SERVO_VALUE,  RIGHT_POSITION_LEFT_SERVO_VALUE));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(LEFT_POSITION_RIGHT_SERVO_VALUE,  RIGHT_POSITION_RIGHT_SERVO_VALUE));

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
            return lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_TURRET;
        }
    }

    public void selectTransition(String desiredLevel){
        switch(desiredLevel){
            case LEFT_POSITION:{
                transitionToPosition(0, 0);
                break;
            } case CENTER_POSITION:{
                transitionToPosition(0.52, 0.52);
                break;
            } case RIGHT_POSITION:{
                transitionToPosition(1, 1);
                break;
            }
        }
    }

    public void transitionToPosition (double leftPosition, double rightPosition) {
        //raising heights to reach different junctions, so four values
        leftTurretServo.setPosition(leftPosition);
        rightTurretServo.setPosition(rightPosition);
    }

    public void centerTurret(){
        transitionToPosition(TURRET_CENTER_POSITION, TURRET_CENTER_POSITION);
    }
}
