package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Map;


public class Lift {
    private Telemetry telemetry;
    public DcMotor liftMotor;

    static final double MM_TO_INCHES = 0.0393700787;

    static final double COUNTS_PER_MOTOR_REV = 28;     // ticks at the motor shaft
    static final double DRIVE_GEAR_REDUCTION = 5.23;     // TODO: Fix to 3:1 gear reduction (slowing down)
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 24.25 * MM_TO_INCHES;     // convert mm to inches
    static final double TICK_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double LIFT_UP_SPEED = 0.5;
    static final double LIFT_DOWN_SPEED = 0.5;

    public final int MINIMUM_CLEARANCE_HEIGHT = 43;    // inches to lift to clear side panels

    public final int LIFT_POSITION_RESET = 0;
    public final int LIFT_POSITION_GROUND = 97;
    public final int LIFT_POSITION_LOWPOLE = 380;
    public final int LIFT_POSITION_MIDPOLE = 590;
    public final int LIFT_POSITION_HIGHPOLE = 860;
    public final int LIFT_POSITION_PICKUP = 8;
    public final int LIFT_ADJUSTMENT = -70;
    public BrainStemRobot robotConstants = null;


    public final double HARD_STOP_CURRENT_DRAW = 100;

    public final String LIFT_SYSTEM_NAME = "Lift";
    public final String LIFT_PICKUP = "PICKUP";
    public final String LIFT_POLE_GROUND = "GROUND";
    public final String LIFT_POLE_LOW = "POLE_LOW";
    public final String LIFT_POLE_MEDIUM = "POlE_MEDIUM";
    public final String LIFT_POLE_HIGH = "POLE_HIGH";
    public final String APPROACH_HEIGHT = "APPROACH_HEIGHT";
    public final String PLACEMENT_HEIGHT = "PLACEMENT_HEIGHT";
    public final String LIFT_SUBHEIGHT = "SUB_HEIGHT";

    public final String TRANSITION_STATE = "TRANSITION";
    public final int DELIVERY_ADJUSTMENT = -3;
    public final int HEIGHT_TOLERANCE = 5;
    public final int CYCLE_TOLERANCE = 25;
    public final String LIFT_CURRENT_STATE = "LIFT CURRENT STATE";

    public static double currentLiftHeight;
    private Map stateMap;


    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        liftMotor = hwMap.dcMotor.get("Lift");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isCollectionHeight() {
        return getPosition() < (LIFT_POSITION_GROUND + CYCLE_TOLERANCE);
    }

    public int getPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void setState() {
        String currentState = getCurrentState();
        String level = (String) stateMap.get(LIFT_SYSTEM_NAME);
        String subheight = (String) stateMap.get(LIFT_SUBHEIGHT);
        telemetry.addData("liftLevel", level);
        telemetry.addData("liftSubHeight", subheight);

        stateMap.put(LIFT_CURRENT_STATE, currentState);
        updateConeCycleState();
        telemetry.addData("liftCurrentState", currentState);
        if (level.equalsIgnoreCase(currentState)) {
            liftMotor.setPower(0);
            return;
        } else {
            selectTransition(level, subheight, currentState);
        }
    }

    private void updateConeCycleState() {
        int position = getStateValue();
        if (isSubheightPlacement()) {
            if (inHeightTolerance(getPosition(), position + LIFT_ADJUSTMENT)) {
                stateMap.put(robotConstants.CYCLE_LIFT_DOWN, robotConstants.STATE_COMPLETE);
            }
        } else{
            if(inHeightTolerance(getPosition(), position)){
                stateMap.put(robotConstants.CYCLE_LIFT_UP, robotConstants.STATE_COMPLETE);
            }
        }
    }




    private boolean isSubheightPlacement(){
        return ((String)stateMap.get(LIFT_SUBHEIGHT)).equalsIgnoreCase(PLACEMENT_HEIGHT);
    }
    private int getStateValue(){
        int position = 0;
        switch((String)stateMap.get(LIFT_CURRENT_STATE)) {
            case LIFT_POLE_HIGH: {
                position = LIFT_POSITION_HIGHPOLE;
                break;
            }
            case LIFT_POLE_MEDIUM: {
                position = LIFT_POSITION_MIDPOLE;
                break;
            }
            case LIFT_POLE_LOW: {
                position = LIFT_POSITION_LOWPOLE;
                break;
            }
            case LIFT_POLE_GROUND:{
                position = LIFT_POSITION_GROUND;
                break;
            }
        }
            return position;
    }


    private void selectTransition(String desiredLevel, String subheight, String currentState){
        switch(desiredLevel){
            case LIFT_PICKUP:{
                transitionToLiftPosition(LIFT_POSITION_PICKUP);
                break;
            }
            case LIFT_POLE_LOW:{
                transitionToLiftPosition(LIFT_POSITION_LOWPOLE + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_MEDIUM:{
                transitionToLiftPosition(LIFT_POSITION_MIDPOLE + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_HIGH:{
                transitionToLiftPosition(LIFT_POSITION_HIGHPOLE + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_GROUND:{
                transitionToLiftPosition(LIFT_POSITION_GROUND + deliveryHeight(subheight));
                break;
            }
        }

    }
    private void transitionToLiftPosition(int ticks){
        raiseHeightTo(ticks);
    }

    public String getCurrentState() {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();
        telemetry.addData("CurrentMotorEncoderTicks", liftMotor.getCurrentPosition());
        telemetry.addData("CurrentPosition", currentPosition);
        if(inHeightTolerance(currentPosition, LIFT_POSITION_GROUND)){
            state = LIFT_POLE_GROUND;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_LOWPOLE)) {
            state = LIFT_POLE_LOW;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_MIDPOLE)) {
            state = LIFT_POLE_MEDIUM;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_HIGHPOLE)) {
            state = LIFT_POLE_HIGH;
        }
        return state;
    }

    public int deliveryHeight(String subheight){
        int height = 0;
        if(subheight.equalsIgnoreCase(PLACEMENT_HEIGHT)){
            height += LIFT_ADJUSTMENT;
        }
        return height;
    }

    public void raiseHeightTo (int heightInTicks) {
        //raising heights to reach different junctions, so four values
        telemetry.addData("raiseHeightCalled" , true);
        liftMotor.setTargetPosition(heightInTicks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
        telemetry.addData("MotorPosition", liftMotor.getCurrentPosition());

    }

    public  boolean isClear () {
        //true means turret can turn and lift is raised to minimum clearance; false is the opposite
        double currentLiftHeight = liftMotor.getCurrentPosition() * TICK_PER_INCH;
        if(currentLiftHeight >= MINIMUM_CLEARANCE_HEIGHT){
            return true;
        }
        return false;

    }
    public void moveToMinHeight(){
        if (!isClear()) {
            raiseHeightTo(MINIMUM_CLEARANCE_HEIGHT);
        }
    }

    public void initializePosition( ) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void setMotor(double power){
//        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(power);
    }

    private boolean inHeightTolerance(double heightPosition, double targetHeight) {
        return (heightPosition > targetHeight - HEIGHT_TOLERANCE) && (heightPosition < targetHeight + HEIGHT_TOLERANCE);
    }

}