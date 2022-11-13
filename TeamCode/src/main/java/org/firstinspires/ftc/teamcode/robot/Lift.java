package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BrainSTEMStateMachine;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.DcMotorEx;


import java.util.ArrayList;
import java.util.Map;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Lift {
  //  private static final class LiftConstants {
//        static final int BOTTOM_ENCODER_TICKS = -5;
//        static final int LOW_POLE__ENCODER_TICKS = 275;
//        static final int MIDDLE_POLE_ENCODER_TICKS = 420;
//        static final int HIGH_POLE_ENCODER_TICKS = 710;
//        static final int JUNCTION_ENCODER_TICKS = 1;
//        static final int COLLECTING_ENCODER_TICKS = 1;
//        static final int LIFT_POSITION_TOLERANCE = 0;
//        static final int MINIMUM_LIFT_TICKS_BElOW_LIFTSTATE_BEFORE_BRAKING = 25;
//        static final int MINIMUM_SAFE_ENCODER_TICKS_FOR_TURRET_TO_TURN = LiftHeight.LOW.getTicks() + 80;
//
//        //motor id's
//        static final String LIFT_MOTOR_1_ID = "Lift-1";
//        static final String LIFT_MOTOR_2_ID = "Lift-2";
//        static final String LIFT_MOTOR_3_ID = "Lift-3";
//        static final String LIFT_MOTOR_4_ID = "Lift-4andOdo";
//
//        //lift motor power
//        static final double STAY_AT_POSITION_POWER = 0.3;
//        static final double GO_UP_LIFT_MOTOR_POWER = 1;
//        static final double GO_DOWN_LIFT_POWER = -0.4;
//
//        //lift motors reversed
//        static final boolean LIFT_MOTOR_1_REVERSED = true;
//        static final boolean LIFT_MOTOR_2_REVERSED = true;
//        static final boolean LIFT_MOTOR_3_REVERSED = true;
//        static final boolean LIFT_MOTOR_4_REVERSED = true;
//
//        //lift motors linear function equation slope
//        static final double LIFT_SLOPE = 0.05;
//        //lift motors linear function y intercept
//        static final double LIFT_Y_INTERCEPT = STAY_AT_POSITION_POWER;
//
//        //PID Constants
//        static final double PROPORTIONAL = 0; //FIXME
//        static final double INTEGRAL = 0; //FIXME
//        static final double DERIVATIVE = 0; //FIXME
//    }

//    public enum LiftHeight {
//        //constants with encoder values
//        DEFAULT(LiftConstants.BOTTOM_ENCODER_TICKS), LOW(LiftConstants.LOW_POLE__ENCODER_TICKS),
//        MIDDLE(LiftConstants.MIDDLE_POLE_ENCODER_TICKS), HIGH(LiftConstants.HIGH_POLE_ENCODER_TICKS),
//        JUNCTION(LiftConstants.JUNCTION_ENCODER_TICKS), COLLECTING(LiftConstants.COLLECTING_ENCODER_TICKS);
//
//        private Integer ticks;
//
//        LiftHeight(Integer ticks) {
//            this.ticks = ticks;
//        }
//
//
//        public Integer getTicks() {
//            return this.ticks;
//        }
//    }
//
//    public enum LiftState {
//
//    }


    private Telemetry telemetry;

    //declare lift motors
    private DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;
    private DcMotorEx liftMotor3;
    private DcMotorEx liftMotor4;

    static final double MM_TO_INCHES = 0.0393700787;

    static final double COUNTS_PER_MOTOR_REV = 28;     // ticks at the motor shaft
    static final double DRIVE_GEAR_REDUCTION = 5.23;     // TODO: Fix to 3:1 gear reduction (slowing down)
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 24.25 * MM_TO_INCHES;     // convert mm to inches
    static final double TICK_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double LIFT_UP_SPEED = 1.0;
    static final double LIFT_DOWN_SPEED = -0.1;

    public final int MINIMUM_CLEARANCE_HEIGHT = 43;    // inches to lift to clear side panels

    public final int LIFT_POSITION_RESET = 0;

    public final int LIFT_HEIGHT_GROUND = 17;
    public final int LIFT_POSITION_LOWPOLE = 380;
    public final int LIFT_POSITION_MIDPOLE = 590;
    public final int LIFT_POSITION_HIGHPOLE = 690;
    public final int LIFT_POSITION_PICKUP = 8;
    public final int LIFT_ADJUSTMENT = -40;
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
    public static double Kv = 0.17;
    public static double Ka = 0.01;
    private Map stateMap;

    PIDFController pidfController = new PIDFController(new PIDCoefficients(Kp, Ki, Kd), Kv, Ka);

    public static double currentLiftHeight;

    //declaring the state map
    private Map stateMap;

    //declaring list of lift motors
    private ArrayList<DcMotor> liftMotors;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry, Map stateMap) {
        //initializing telemetry
        this.telemetry = telemetry;
        //initializing state map
        this.stateMap = stateMap;

        //initialize lift motors
        liftMotor1 = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Lift-1"));
        liftMotor2 = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Lift-2"));
        liftMotor3 = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Lift-3"));
        liftMotor4 = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Lift-4andOdo"));

        liftMotors = new ArrayList<>();

        //add lift motors to list
        liftMotors.add(liftMotor1);
        liftMotors.add(liftMotor2);
        liftMotors.add(liftMotor3);
        liftMotors.add(liftMotor4);

        liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting motor modes
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setting directions
        /*liftMotor1.setDirection(LiftConstants.LIFT_MOTOR_1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        liftMotor2.setDirection(LiftConstants.LIFT_MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        liftMotor3.setDirection(LiftConstants.LIFT_MOTOR_3_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        liftMotor4.setDirection(LiftConstants.LIFT_MOTOR_4_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);*/


        initializeLiftMotor(liftMotor1);
        initializeLiftMotor(liftMotor2);
        initializeLiftMotor(liftMotor3);
        initializeLiftMotor(liftMotor4);

        pidfController.setInputBounds(0, LIFT_POSITION_HIGHPOLE);
        pidfController.setOutputBounds(0, 1);
    }

    private void initializeLiftMotor(DcMotorEx liftMotor) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveUp() {
        setMotorsPower(LIFT_UP_SPEED);
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
        return getPosition() < (LIFT_HEIGHT_GROUND + CYCLE_TOLERANCE);
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
        if (shouldLiftMove(level, currentState)) {
            selectTransition(level, subheight, currentState);
        } else {
            liftMotor3.setPower(0);
        }
    }

    private boolean shouldLiftMove(String level, String currentState) {
        return ((String) stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                !level.equalsIgnoreCase(currentState);
    }

    private void updateConeCycleState() {
        int position = getStateValue();
        if (isCycleInProgress(constants.CYCLE_LIFT_DOWN) && isSubheightPlacement()) {
            if (inHeightTolerance(getPosition(), position + LIFT_ADJUSTMENT)) {
                stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_COMPLETE);
            }
        } else if (isCycleInProgress(constants.CYCLE_LIFT_UP) && inHeightTolerance(getPosition(), position)) {
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
        }
    }

    private boolean isCycleInProgress(String cycleName) {
        return ((String) stateMap.get(cycleName)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }

    private boolean isSubheightPlacement() {
        return ((String) stateMap.get(LIFT_SUBHEIGHT)).equalsIgnoreCase(PLACEMENT_HEIGHT);
    }

    private int getStateValue() {
        int position = 0;
        switch ((String) stateMap.get(LIFT_CURRENT_STATE)) {
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
            case LIFT_POLE_GROUND: {

                position = LIFT_HEIGHT_GROUND;
                break;
            }
        }
        return position;
    }


    private void selectTransition(String desiredLevel, String subheight, String currentState) {
        switch (desiredLevel) {
            case LIFT_POLE_LOW: {
                transitionToLiftPosition(LIFT_POSITION_LOWPOLE + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_MEDIUM: {
                transitionToLiftPosition(LIFT_POSITION_MIDPOLE + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_HIGH: {
                transitionToLiftPosition(LIFT_POSITION_HIGHPOLE + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_GROUND: {

                transitionToLiftPosition(LIFT_HEIGHT_GROUND + deliveryHeight(subheight));
                break;
            }
        }

    }

    private void transitionToLiftPosition(int ticks) {
        setHeightTo(ticks);
    }

    public String getCurrentState(String subheight) {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();

        if (inHeightTolerance(currentPosition, LIFT_HEIGHT_GROUND + deliveryHeight(subheight))) {
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

    public int deliveryHeight(String subheight) {
        int height = 0;
        if (subheight.equalsIgnoreCase(PLACEMENT_HEIGHT)) {
            height += LIFT_ADJUSTMENT;
        }
        return height;
    }

    public void raiseHeightTo(int heightInTicks) {
        //raising heights to reach different junctions, so four values
        telemetry.addData("raiseHeightCalled", true);
        telemetry.addData("heightInTicks", heightInTicks);

        int position = getPosition();
        telemetry.addData("position", position);

        // calculate the error
        int error = heightInTicks - position;
        double direction = 0;

        if (Math.abs(error) > 0) {
            if (heightInTicks > position) {
                direction = 1.65;
            } else {
                direction = 0.46;
            }
        } else {
            if (heightInTicks < 400) {
                direction = 0.25;
            } else {
                direction = 0.3;
            }

    public void setHeightTo(int heightInTicks) {
        //raising heights to reach different junctions, so four values
        telemetry.addData("raiseHeightCalled", true);
        telemetry.addData("heightInTicks", heightInTicks);
        int currentPosition = getPosition();
        int goDown;
        if (getPosition() > heightInTicks + 25) {
            goDown = -1;
        } else if (getPosition() < heightInTicks - 25) {
            goDown = 1;
        } else {
            goDown = 0;

        }
        /*if (stateMap.get(constants.CONE_CYCLE).equals(constants.STATE_IN_PROGRESS)) {
            setMotorPowerFromDistance(goDown * Math.abs(currentPosition - heightInTicks) * 5);
        } else*/
        setMotorPowerFromDistance(goDown * Math.abs(currentPosition - heightInTicks), heightInTicks);

        double power = ((Kp * direction * error) / LIFT_POSITION_HIGHPOLE) + Kv;
        setMotorsPower(power);

        telemetry.addData("PID Correction", power);
    }

    public void setMotor(double power) {
//        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor3.setPower(power);
    }

    public void setAllMotorPowers(double power) {
        for (DcMotor liftMotor : liftMotors) {
            liftMotor.setPower(power);
        }
    }

    public void setAllMotorPowersExcept3(double power){
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
        liftMotor4.setPower(power);

    }

    public ArrayList<Double> getLiftMotorPowers() {
        ArrayList<Double> liftMotorPowers = new ArrayList<>();
        for (DcMotor liftMotor : liftMotors) {
            liftMotorPowers.add(liftMotor.getPower());
        }
        return liftMotorPowers;
    }

    public void setMotorPowerFromDistance(int distanceFromDesiredHeight, int heightInTicks) {
        if (heightInTicks < 100){
            setAllMotorPowersExcept3(0.0);
            liftMotor3.setTargetPosition(heightInTicks);
            liftMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (Math.abs(distanceFromDesiredHeight) < 3) {
                telemetry.addData("Current Lift Position: ", getPosition());
                liftMotor3.setPower(0.0);
            } else {
                liftMotor3.setPower(1.0);
            }
        } else if (distanceFromDesiredHeight > 0) {
            liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (distanceFromDesiredHeight > 200) {
                setAllMotorPowers(1);
            } else if (distanceFromDesiredHeight > 100) {
                setAllMotorPowers(0.8);
            }
        } else if (distanceFromDesiredHeight < 0) {
            liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (Math.abs(distanceFromDesiredHeight) > 200) {
                setAllMotorPowers(-0.1);
            } else if (Math.abs(distanceFromDesiredHeight) > 100) {
                setAllMotorPowers(0.01);
            }
        } else {
            setAllMotorPowers(0.36);
        }


        /*if (distanceFromDesiredHeight > 0) {
            if (Math.abs(distanceFromDesiredHeight) > 200) {
                setAllMotorPowers(1);
            } else if (Math.abs(distanceFromDesiredHeight) > 100) {
                setAllMotorPowers(0.8);
                //setAllMotorPowers(0.05 * (Math.pow(0.25 * distanceFromDesiredHeight, 0.578)) + 0.3);
            } else {
                setAllMotorPowers(0.4);
            }
        } else if (distanceFromDesiredHeight < 0) {
            telemetry.addData("distanceFromDesiredHeight", distanceFromDesiredHeight);
            if (Math.abs(distanceFromDesiredHeight) > 200) {
                setAllMotorPowers(-0.1);
            } else if (Math.abs(distanceFromDesiredHeight) > 100 && stateMap.get(constants.CONE_CYCLE).equals(constants.STATE_IN_PROGRESS)) {
                if (Math.abs(distanceFromDesiredHeight) > 50) {
                    setAllMotorPowers(-0.07);
                } else {
                    setAllMotorPowers(0);
                }
                //setAllMotorPowers(-0.05 * (Math.pow(6 * (distanceFromDesiredHeight+200), 0.578) + 0.55));
            } else if (Math.abs(distanceFromDesiredHeight) > 100) {
                setAllMotorPowers(-0.05);
                if (stateMap.get(constants.CONE_CYCLE).equals(constants.STATE_IN_PROGRESS)) {
                    setAllMotorPowers(-0.07);
                } else {
                    setAllMotorPowers(0.1);
                }
            } else {
                if (getPosition() < 300) {
                    setAllMotorPowers(0.2);
                } else {
                    setAllMotorPowers(0.4);
                }
            }
        } else {
            if (getPosition() < 300) {
                setAllMotorPowers(0.2);
            } else {
                setAllMotorPowers(0.4);
            }
        }*/


    }

    private boolean inHeightTolerance(double heightPosition, double targetHeight) {
        return (heightPosition > targetHeight - HEIGHT_TOLERANCE) && (heightPosition < targetHeight + HEIGHT_TOLERANCE);
    }

    public boolean isLiftUp() {
        return (getPosition() > LIFT_POSITION_GROUND);

        return (getPosition() > LIFT_HEIGHT_GROUND);
    }

    public void moveUp(int position) {
        int targetPosition = liftMotor3.getCurrentPosition() + position;
        liftMotor3.setTargetPosition(targetPosition);
        liftMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor3.setPower(1.0);
    }

    public void moveDown(int liftDownIncrement) {
        if (liftDownIncrement < 7) {
            setAllMotorPowers(-0.25);
        } else if (liftDownIncrement < 14) {
            setAllMotorPowers(0.6);
        } else if (liftDownIncrement < 18) {
            setAllMotorPowers(0.3);
        }

        telemetry.addData("liftDownIncrement", liftDownIncrement);
    }

}