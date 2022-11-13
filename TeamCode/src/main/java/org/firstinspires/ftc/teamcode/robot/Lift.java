package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
import com.acmerobotics.dashboard.config.Config;

@Config
public class Lift {
    private Telemetry telemetry;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;
    public DcMotor liftMotor3;
    public DcMotor liftMotor4;

    static final double MM_TO_INCHES = 0.0393700787;

    static final double COUNTS_PER_MOTOR_REV = 28;     // ticks at the motor shaft
    static final double DRIVE_GEAR_REDUCTION = 5.23;     // TODO: Fix to 3:1 gear reduction (slowing down)
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 24.25 * MM_TO_INCHES;     // convert mm to inches
    static final double TICK_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double LIFT_UP_SPEED = 1.0;
    static final double LIFT_DOWN_SPEED = -0.1;

    public final int MINIMUM_CLEARANCE_HEIGHT = 43;    // inches to lift to clear side panels

    public final int LIFT_POSITION_RESET = 0;
    public final int LIFT_POSITION_GROUND = 50;
    public final int LIFT_POSITION_LOWPOLE = 380;
    public final int LIFT_POSITION_MIDPOLE = 590;
    public final int LIFT_POSITION_HIGHPOLE = 600;
    public final int LIFT_POSITION_PICKUP = 8;
    public final int LIFT_ADJUSTMENT = -25;
    Constants constants = new Constants();


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
    public final int HEIGHT_TOLERANCE = 15;
    public final int CYCLE_TOLERANCE = 5;
    public final String LIFT_CURRENT_STATE = "LIFT CURRENT STATE";

    static final String LIFT_MOTOR_1_ID = "Lift-1";
    static final String LIFT_MOTOR_2_ID = "Lift-2";
    static final String LIFT_MOTOR_3_ID = "Lift-3";
    static final String LIFT_MOTOR_4_ID = "Lift-4andOdo";

    public static double Kp = 2.0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kv = 0.2;
    public static double Ka = 0.01;
    private Map stateMap;

    PIDFController pidfController = new PIDFController(new PIDCoefficients(Kp, Ki, Kd), Kv, Ka);

    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        liftMotor1 = hwMap.dcMotor.get(LIFT_MOTOR_1_ID);
        liftMotor2 = hwMap.dcMotor.get(LIFT_MOTOR_2_ID);
        liftMotor3 = hwMap.dcMotor.get(LIFT_MOTOR_3_ID);
        liftMotor4 = hwMap.dcMotor.get(LIFT_MOTOR_4_ID);

        initializeLiftMotor(liftMotor1);
        initializeLiftMotor(liftMotor2);
        initializeLiftMotor(liftMotor3);
        initializeLiftMotor(liftMotor4);

        pidfController.setInputBounds(0, LIFT_POSITION_HIGHPOLE);
        pidfController.setOutputBounds(0, 1);
    }

    private void initializeLiftMotor(DcMotor liftMotor) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveUp() {
        setMotorsPower(LIFT_UP_SPEED);
    }

    public void moveDown() {
        setMotorsPower(LIFT_DOWN_SPEED);
    }

    public void stop() {
        setMotorsPower(0.0);
    }

    private void setMotorsPower(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
        liftMotor3.setPower(power);
        liftMotor4.setPower(power);
    }

    private void setBoostPower(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
        liftMotor4.setPower(power);
    }

    public boolean isCollectionHeight() {
        return getPosition() < (LIFT_POSITION_GROUND + CYCLE_TOLERANCE);
    }

    public int getPosition() {
        return liftMotor3.getCurrentPosition();
    }

    public void setState() {
        String subheight = (String) stateMap.get(LIFT_SUBHEIGHT);
        String currentState = getCurrentState(subheight);
        String level = (String) stateMap.get(LIFT_SYSTEM_NAME);

        stateMap.put(LIFT_CURRENT_STATE, currentState);

        updateConeCycleState();
        telemetry.addData("liftCurrentState", currentState);
        if (shouldLiftMove(level, currentState) ) {
            selectTransition(level, subheight, currentState);
        } else {
            liftMotor3.setPower(0);
        }
    }

    private boolean shouldLiftMove(String level, String currentState) {
        return ((String)stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                ((String)stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                !level.equalsIgnoreCase(currentState);
    }

    private void updateConeCycleState() {
        int position = getStateValue();
        if (isCycleInProgress(constants.CYCLE_LIFT_DOWN) && isSubheightPlacement()) {
            if (inHeightTolerance(getPosition(), position + LIFT_ADJUSTMENT)) {
                stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_COMPLETE);
            }
        } else if(isCycleInProgress(constants.CYCLE_LIFT_UP) && inHeightTolerance(getPosition(), position)){
                stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
        }
    }

    private boolean isCycleInProgress(String cycleName) {
        return ((String)stateMap.get(cycleName)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
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

    public String getCurrentState(String subheight) {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();
        if(inHeightTolerance(currentPosition, LIFT_POSITION_GROUND + deliveryHeight(subheight))){
            state = LIFT_POLE_GROUND;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_LOWPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_LOW;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_MIDPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_MEDIUM;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_HIGHPOLE + deliveryHeight(subheight))) {
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
        telemetry.addData("heightInTicks" , heightInTicks);

        int position = getPosition();
        telemetry.addData("position" , position);

        // calculate the error
        int error = heightInTicks - position;
        double direction = 0;
        if (error > 0) {
            direction = 1;
        } else {
            direction = 0.3;
        }

        double power = ((Kp * direction * error) / LIFT_POSITION_HIGHPOLE) + Kv;
        setMotorsPower(power);

        telemetry.addData("PID Correction", power);
    }

    public void setMotor(double power){
//        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor3.setPower(power);
    }

    private boolean inHeightTolerance(double heightPosition, double targetHeight) {
        return (heightPosition > targetHeight - HEIGHT_TOLERANCE) && (heightPosition < targetHeight + HEIGHT_TOLERANCE);
    }

    public boolean isLiftUp(){
        return (getPosition() > LIFT_POSITION_GROUND);
    }

}