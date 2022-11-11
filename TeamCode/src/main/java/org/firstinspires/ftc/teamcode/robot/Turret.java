package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

public class Turret {
    public final String     SYSTEM_NAME = "TURRET";
    public final String     LEFT_POSITION = "LEFT_STATE";
    public final String     RIGHT_POSITION = "RIGHT_STATE";
    public final String     CENTER_POSITION = "CENTER_STATE";
    public final double     LEFT_POSITION_LEFT_SERVO_VALUE = 0.01;
    public final double     LEFT_POSITION_RIGHT_SERVO_VALUE = 0.01;
    public final double     CENTER_POSITION_LEFT_SERVO_VALUE = 0.45;
    public final double     CENTER_POSITION_RIGHT_SERVO_VALUE = 0.435;
    public final double     RIGHT_POSITION_LEFT_SERVO_VALUE = 0.9;
    public final double     RIGHT_POSITION_RIGHT_SERVO_VALUE = 0.9;
    public final int        LIFT_MIN_HEIGHT_TO_MOVE_TURRET = 75;

    public Telemetry telemetry;
    private Servo leftTurretServo;
    private Servo rightTurretServo;
    private Map stateMap;

    public Turret(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        leftTurretServo = hwMap.servo.get("turretLeft");
        rightTurretServo = hwMap.servo.get("turretRight");
    }

    public void setState(Lift lift){
        if(isLiftTooLow(lift)){
            return;
        }
        else{
            selectTransition((String) stateMap.get(SYSTEM_NAME));
        }
    }

    public boolean isLiftTooLow(Lift lift) {
        return lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_TURRET;
    }

    private void selectTransition(String desiredLevel){
        switch(desiredLevel){
            case LEFT_POSITION:{
                transitionToPosition(LEFT_POSITION_LEFT_SERVO_VALUE, LEFT_POSITION_RIGHT_SERVO_VALUE);
                break;
            } case CENTER_POSITION:{
                transitionToPosition(CENTER_POSITION_LEFT_SERVO_VALUE, CENTER_POSITION_RIGHT_SERVO_VALUE);
                break;
            } case RIGHT_POSITION:{
                transitionToPosition(RIGHT_POSITION_LEFT_SERVO_VALUE, RIGHT_POSITION_RIGHT_SERVO_VALUE);
                break;
            }
        }
    }

    public void transitionToPosition (double leftPosition, double rightPosition) {
        //raising heights to reach different junctions, so four values
        leftTurretServo.setPosition(leftPosition);
        rightTurretServo.setPosition(rightPosition);
    }
}
