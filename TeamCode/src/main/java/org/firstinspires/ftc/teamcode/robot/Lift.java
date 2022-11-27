package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import java.util.Map;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Lift {

    private static final class LiftConstants {
        //encoder positions
        static final int BOTTOM_ENCODER_TICKS = 0;
        static final int LOW_POLE_ENCODER_TICKS = 330;
        static final int MIDDLE_POLE_ENCODER_TICKS = 530;
        static final int HIGH_POLE_ENCODER_TICKS = 720;
        static final int JUNCTION_ENCODER_TICKS = 1; //FIXME
        static final int COLLECTING_ENCODER_TICKS = 25;
        static final int LIFT_POSITION_TOLERANCE = 8;
        static final int CONE_CYCLE_POSITION_TOLERANCE = 3;

        //motor id's
        static final String LIFT_MOTOR_1_ID = "Lift-1";
        static final String LIFT_MOTOR_2_ID = "Lift-2";
        static final String LIFT_MOTOR_3_ID = "Lift-3";
        static final String LIFT_MOTOR_4_ID = "Lift-4";

        //lift motor power
        static final double STAY_AT_POSITION_BOTTOM_POWER = 0.1;
        static final double STAY_AT_POSITION_TOP_POWER = 0.45;
        static final double GO_UP_LIFT_MOTOR_POWER = 1;
        static final double GO_UP_SLOW_LIFT_POWER = 0.8;
        static final double GO_DOWN_LIFT_POWER = -1;
        static final double GO_UP_CONE_CYCLE_BOTTOM_POWER = 1;
        static final double GO_DOWN_CONE_CYCLE_BOTTOM_POWER = -1;
        static final double GO_UP_CONE_CYCLE_TOP_POWER = 1;
        static final double GO_DOWN_CONE_CYCLE_TOP_POWER = -1;

        //lift motors reversed
        static final boolean LIFT_MOTOR_1_REVERSED = false;
        static final boolean LIFT_MOTOR_2_REVERSED = false;
        static final boolean LIFT_MOTOR_3_REVERSED = false;
        static final boolean LIFT_MOTOR_4_REVERSED = false;
    }

    public enum LiftHeight {
        //constants with encoder values
        BOTTOM(LiftConstants.BOTTOM_ENCODER_TICKS), LOW(LiftConstants.LOW_POLE_ENCODER_TICKS),
        MIDDLE(LiftConstants.MIDDLE_POLE_ENCODER_TICKS), HIGH(LiftConstants.HIGH_POLE_ENCODER_TICKS),
        JUNCTION(LiftConstants.JUNCTION_ENCODER_TICKS), COLLECTING(LiftConstants.COLLECTING_ENCODER_TICKS),
        TRANSITIONING(null);

        private Integer ticks;

        LiftHeight(Integer ticks) {
            this.ticks = ticks;
        }


        public Integer getTicks() {
            return this.ticks;
        }

    }

    public enum LiftActivation {
        ACTIVATED, DEACTIVATED
    }

    //telemetry
    private Telemetry telemetry;

    //lift height state
    private LiftHeight desiredLiftHeight = LiftHeight.COLLECTING;

    //lift action state
    private LiftActivation liftActivation = LiftActivation.DEACTIVATED;

    //declare lift motors
    public DcMotorEx liftMotor1;
    public DcMotorEx liftMotor2;
    public DcMotorEx liftMotor3;
    public DcMotorEx liftMotor4;


    static final double LIFT_UP_SPEED = 1.0;
    static final double LIFT_DOWN_SPEED = -0.1;

    public final int MINIMUM_CLEARANCE_HEIGHT = 43;    // inches to lift to clear side panels

    public final int LIFT_POSITION_RESET = 0;
    public final int LIFT_POSITION_GROUND = 25;
    public final int LIFT_POSITION_LOWPOLE = 330;
    public final int LIFT_POSITION_MIDPOLE = 530;
    public int LIFT_POSITION_HIGHPOLE = 720;
    public final int LIFT_POSITION_PICKUP = 1;
    public final int LIFT_ADJUSTMENT = -30;
    public final int CYCLE_LIFT_DOWN_TIME_BOTTOM = 300;
    public final int CYCLE_LIFT_UP_TIME_BOTTOM = 300;
    public final int CYCLE_LIFT_DOWN_TIME_TOP = 50;
    public final int CYCLE_LIFT_UP_TIME_TOP = 250;
    public final int LIFT_FINE_UP = 25;
    public final int LIFT_FINE_DOWN = 25;

    public final int LIFT_POSITION_AUTO_RESTING = 250;
    public final int LIFT_POSITION_AUTO_CYCLE_1 = 110;
    public final int LIFT_POSITION_AUTO_CYCLE_2 = 76;
    public final int LIFT_POSITION_AUTO_CYCLE_3 = 55;
    public final int LIFT_POSITION_AUTO_CYCLE_4 = 22;
    public final int LIFT_POSITION_AUTO_CYCLE_5 = 2;



    Constants constants = new Constants();


    public boolean coneCycleNowAt;


    public final double HARD_STOP_CURRENT_DRAW = 100;

    public final String LIFT_SYSTEM_NAME = "Lift";
    public final String LIFT_PICKUP = "PICKUP";
    public final String LIFT_POLE_GROUND = "GROUND";
    public final String LIFT_POLE_LOW = "POLE_LOW";
    public final String LIFT_POLE_MEDIUM = "POlE_MEDIUM";
    public final String LIFT_POLE_HIGH = "POLE_HIGH";
    public final String LIFT_TARGET_HEIGHT = "LIFT TARGET HEIGHT";
    public final String APPROACH_HEIGHT = "APPROACH_HEIGHT";
    public final String PLACEMENT_HEIGHT = "PLACEMENT_HEIGHT";
    public final String LIFT_SUBHEIGHT = "SUB_HEIGHT";
    public final String LIFT_FINEADJ_DOWN = "LIFT_FINEADJ_DOWN";
    public final String LIFT_FINEADJ_UP = "LIFT_FINEADJ_UP";
    public final String LIFT_CYCLE_COLLECT_1 = "LIFT_CYCLE_COLLECT_1";
    public final String LIFT_CYCLE_COLLECT_2 = "LIFT_CYCLE_COLLECT_2";
    public final String LIFT_CYCLE_COLLECT_3 = "LIFT_CYCLE_COLLECT_3";
    public final String LIFT_CYCLE_COLLECT_4 = "LIFT_CYCLE_COLLECT_4";
    public final String LIFT_CYCLE_COLLECT_5 = "LIFT_CYCLE_COLLECT_5";
    public final String LIFT_RESTING_IN_AUTO = "LIFT_RESTING_IN_AUTO";

    public final String TRANSITION_STATE = "TRANSITION";
    public final int DELIVERY_ADJUSTMENT = -3;
    public final int HEIGHT_TOLERANCE = 3;
    public final int CYCLE_TOLERANCE = 5;
    public final String LIFT_CURRENT_STATE = "LIFT CURRENT STATE";

    //declaring list of lift motors
    private ArrayList<DcMotor> liftMotors;

    //is lift going down variable
    private boolean isLiftGoingDown;

    //autonomous variable
    private boolean isAuto;

    private Map stateMap;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry, Map stateMap) {

        //telemetry
        this.telemetry = telemetry;

        this.stateMap = stateMap;

        //initialize lift motors
        liftMotor1 = new CachingMotor(hardwareMap.get(DcMotorEx.class, LiftConstants.LIFT_MOTOR_1_ID));
        liftMotor2 = new CachingMotor(hardwareMap.get(DcMotorEx.class, LiftConstants.LIFT_MOTOR_2_ID));
        liftMotor3 = new CachingMotor(hardwareMap.get(DcMotorEx.class, LiftConstants.LIFT_MOTOR_3_ID));
        liftMotor4 = new CachingMotor(hardwareMap.get(DcMotorEx.class, LiftConstants.LIFT_MOTOR_4_ID));

        initializeLiftMotor(liftMotor1);
        initializeLiftMotor(liftMotor2);
        initializeLiftMotor(liftMotor3);
        initializeLiftMotor(liftMotor4);


        //creating list of lift motors for iteration
        liftMotors = new ArrayList<>();

        //add lift motors to list
        liftMotors.add(liftMotor1);
        liftMotors.add(liftMotor2);
        liftMotors.add(liftMotor3);
        liftMotors.add(liftMotor4);

        //setting lift behaviors
        for(DcMotor liftMotor : liftMotors){
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //setting directions
        liftMotor1.setDirection(LiftConstants.LIFT_MOTOR_1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        liftMotor2.setDirection(LiftConstants.LIFT_MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        liftMotor3.setDirection(LiftConstants.LIFT_MOTOR_3_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        liftMotor4.setDirection(LiftConstants.LIFT_MOTOR_4_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void initializeLiftMotor(DcMotor liftMotor) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void setAllMotorPowers(double power) {
        for (DcMotor liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(power);
        }
    }

    private void setMotorsPower(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
        liftMotor3.setPower(power);
        liftMotor4.setPower(power);
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
        if (shouldLiftMove(level, currentState)) {
            if (stateMap.get(constants.LIFT_COMPLETE_TIME) == "0") {
                stateMap.put(constants.LIFT_START_TIME, String.valueOf(System.currentTimeMillis()));
            }
            selectTransition(level, subheight, currentState);
        } else {
            stateMap.put(constants.LIFT_COMPLETE_TIME, "0");
            stateMap.put(constants.LIFT_INTEGRAL_SUM, "0.0");
            setAllMotorPowers(heightFactor(getPosition()));
        }
    }

    public boolean haveLiftMotorsReachedTheirDesiredPositions() {
        for (DcMotor liftMotor : liftMotors) {
            int currentPosition = liftMotor.getCurrentPosition();
            int desiredPosition = liftMotor.getTargetPosition();
            if (!(currentPosition <= desiredPosition + 4 && currentPosition >= desiredPosition - 4)) {
                return false;
            }
        }
        return true;
    }

    private boolean shouldLiftMove(String level, String currentState) {
        return ((String) stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                !level.equalsIgnoreCase(currentState);
    }

    private void updateConeCycleState() {
        if (getPosition() > 400) {
            int position = getStateValue();
            if (isCycleInProgress(constants.CYCLE_LIFT_DOWN) && isSubheightPlacement()) {
                if (haveLiftMotorsReachedTheirDesiredPositions() || isCycleExpired(CYCLE_LIFT_DOWN_TIME_TOP)) {
                    stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_COMPLETE);
                }
            } else if (isCycleInProgress(constants.CYCLE_LIFT_UP) && (haveLiftMotorsReachedTheirDesiredPositions() || isCycleExpired(CYCLE_LIFT_UP_TIME_TOP))) {
                stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
            }
        } else {
            int position = getStateValue();
            if (isCycleInProgress(constants.CYCLE_LIFT_DOWN) && isSubheightPlacement()) {
                if (getPosition() < position + LIFT_ADJUSTMENT || isCycleExpired(CYCLE_LIFT_DOWN_TIME_BOTTOM)) {
                    stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_COMPLETE);
                }
            } else if (isCycleInProgress(constants.CYCLE_LIFT_UP) && (getPosition() > position || isCycleExpired(CYCLE_LIFT_UP_TIME_BOTTOM))) {
                stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
            }
        }

    }

    private boolean isCycleExpired(int cycleTime) {
        return (System.currentTimeMillis() > Long.valueOf(String.valueOf(stateMap.get(constants.CONE_CYCLE_START_TIME))) + cycleTime);
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
                position = LIFT_POSITION_GROUND;
                break;
            }
            case LIFT_FINEADJ_UP: {
                position = getAvgLiftPosition() + LIFT_FINE_UP;
                break;
            }
            case LIFT_FINEADJ_DOWN: {
                position = getAvgLiftPosition() - LIFT_FINE_DOWN;
                break;
            }
            case LIFT_RESTING_IN_AUTO: {
                position = LIFT_POSITION_AUTO_RESTING;
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
                transitionToLiftPosition(LIFT_POSITION_GROUND + deliveryHeight(subheight));
                break;
            }
            case LIFT_FINEADJ_UP: {
                transitionToLiftPosition(getAvgLiftPosition() + deliveryHeight(subheight));
                break;
            }
            case LIFT_FINEADJ_DOWN: {
                transitionToLiftPosition(getAvgLiftPosition() - deliveryHeight(subheight));
                break;
            }
            case LIFT_RESTING_IN_AUTO: {
                transitionToLiftPosition(LIFT_POSITION_AUTO_RESTING + deliveryHeight(subheight));
            }
        }

    }

    private void transitionToLiftPosition(int ticks) {
        raiseHeightTo(ticks);
    }

    public String getCurrentState(String subheight) {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();
        if (inHeightTolerance(currentPosition, LIFT_POSITION_GROUND + deliveryHeight(subheight))) {
            state = LIFT_POLE_GROUND;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_LOWPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_LOW;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_MIDPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_MEDIUM;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_HIGHPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_HIGH;
        }
        else if (inHeightTolerance(currentPosition, LIFT_FINE_UP + deliveryHeight(subheight))) {
            state = LIFT_FINEADJ_UP;
        } else if (inHeightTolerance(currentPosition, LIFT_FINE_DOWN + deliveryHeight(subheight))) {
            state = LIFT_FINEADJ_DOWN;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_AUTO_RESTING + deliveryHeight(subheight))) {
            state = LIFT_RESTING_IN_AUTO;
        }
        return state;
    }

    public int deliveryHeight(String subheight) {
        int height = 0;
        if (subheight.equalsIgnoreCase(PLACEMENT_HEIGHT)) {
            height += LIFT_ADJUSTMENT + 25;
        }
        return height;
    }

    private double heightFactor(int heightInTicks) {
        double factor = Math.abs(0.0003 * heightInTicks) + 0.15;
        telemetry.addData("heightFactor", factor);
        return factor;
    }

    private double errorToPowerLookup(int error, int heightInTicks) {
        double power = 0.0;
        if (error > 25) {
            power = 1.0;
        } else if (error <= 25 && error > 3) {
            power = Math.min(heightFactor(heightInTicks) + 0.5, 0.75);
        } else if (error < -200) {
            power = -0.1;
        } else if (error > -200 && error < -50) {
            power = -0.01;
        } else if (error >= -50 && error < -7) {
            power = -0.0;
        } else if (error >= -7 && error < -3) {
            power = 0.0;
        } else {
            power = heightFactor(heightInTicks);
        }

        return power;
    }

    private void runAllMotorsToPosition(int position, double power) {
        for (DcMotor liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(position);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(power);
        }
    }

    public void raiseHeightTo(int heightInTicks) {
//        long liftStartTime = Long.valueOf(String.valueOf(stateMap.get(constants.LIFT_START_TIME)));
//        long elapsedTime = System.currentTimeMillis() - liftStartTime;

        //raising heights to reach different junctions, so four values
        telemetry.addData("raiseHeightCalled", true);
        telemetry.addData("heightInTicks", heightInTicks);
//        telemetry.addData("liftStartTime", liftStartTime);
//        telemetry.addData("elapsedTime", elapsedTime);

        //int position = getPosition();

        int position = getAvgLiftPosition();

//        double integralSum = Double.valueOf((String) stateMap.get(constants.LIFT_INTEGRAL_SUM));
        telemetry.addData("position", position);

        // calculate the error
        int error = heightInTicks - position;
//        double direction = 0;
//        integralSum = integralSum + (error * elapsedTime);

//        stateMap.put(constants.LIFT_INTEGRAL_SUM, String.valueOf(integralSum));

//        if (heightInTicks > position) {
//            direction = 1.0;
//        } else {
//            direction = 0.07;
//        }

//        double power = ((Kp * direction * error) / LIFT_POSITION_HIGHPOLE) +
//                (heightFactor(heightInTicks) * Kv) +
//                (integralSum * direction * Ki);
        //double power = 0;
//
        telemetry.addData("ERROR", error);
        telemetry.addData("Current Positions: ", getLiftPositions());
        telemetry.addData("Lift Power (M1): ", liftMotor1.getPower());
        telemetry.addData("Lift Power (M2): ", liftMotor2.getPower());
        telemetry.addData("Lift Power (M3): ", liftMotor3.getPower());
        telemetry.addData("Lift Power (M4): ", liftMotor4.getPower());
        //double power = errorToPowerLookup(error, heightInTicks);

        //runAllMotorsToPosition(400, 0.8);

        telemetry.addData("Target Positions: ", getLiftTargetPositions());
        telemetry.update();


        if (isCycleInProgress(constants.CYCLE_LIFT_DOWN)) {
            telemetry.addData("Setting Raw Power; ", "NOOOO");
            telemetry.addData("Using Run To Position; ", "YESSSS");
            telemetry.update();
            runAllMotorsToPosition(heightInTicks + LIFT_ADJUSTMENT, 1);
            //setAllMotorPowers(-0.3);
        } else if (isCycleInProgress(constants.CYCLE_LIFT_UP)) {
            telemetry.addData("Setting Raw Power; ", "NOOOO");
            telemetry.addData("Using Run To Position; ", "YESSSS");
            telemetry.update();
            runAllMotorsToPosition(heightInTicks, 1);
            //setAllMotorPowers(1);
        } else if (position >= heightInTicks - 8 && position <= heightInTicks + 8) {
            if (heightInTicks > 400) {
                telemetry.addData("Setting Raw Power; ", "YESS");
                telemetry.addData("Using Run To Position; ", "NOO");
                telemetry.update();
                setAllMotorPowers(0.45);
            } else {
                telemetry.addData("Setting Raw Power; ", "YESS");
                telemetry.addData("Using Run To Position; ", "NOOO");
                telemetry.update();
                setAllMotorPowers(0.1);
            }
        } else if (position > heightInTicks) {
            if (position > heightInTicks + 200) {
                telemetry.addData("Setting Raw Power; ", "YES");
                telemetry.addData("Using Run To Position; ", "NO");
                telemetry.update();
                setAllMotorPowers(-0.1);
            } else if  (position < 35 && heightInTicks < 35 ) {
                telemetry.addData("Setting Raw Power; ", "YES");
                telemetry.addData("Using Run To Position; ", "NO");
                telemetry.update();
                setAllMotorPowers(0);
            }else {
                telemetry.addData("Setting Raw Power; ", "NOOOO");
                telemetry.addData("Using Run To Position; ", "YESSSS");
                telemetry.update();
                runAllMotorsToPosition(heightInTicks, 1);
            }
        } else {
            if (position < heightInTicks - 3) {
                telemetry.addData("Setting Raw Power; ", "YESSS");
                telemetry.addData("Using Run To Position; ", "NOOOO");
                telemetry.update();
                setAllMotorPowers(1);
            } else {
                telemetry.addData("Using Run To Position; ", "YESSS");
                telemetry.addData("Setting Raw Power; ", "NOOOOO");
                telemetry.update();
                runAllMotorsToPosition(heightInTicks, 1);
            }
        }


        //setMotorsPower(power);

        telemetry.addData("Lift Motor Powers: ", getLiftMotorPowers());
        //setAllMotorPowers(1);
    }

    public int getAvgLiftPosition(){
        double positionSum = 0;
        for(DcMotor liftMotor : liftMotors){
            positionSum += liftMotor.getCurrentPosition();
        }
        return (int)(positionSum/liftMotors.size());
    }

    public void resetAllLiftMotorEncoders(){
        for(DcMotor liftMotor : liftMotors){
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    public void setMotor(double power) {
//        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor3.setPower(power);
    }

    public boolean inHeightTolerance(double heightPosition, double targetHeight) {
        return (heightPosition > targetHeight - HEIGHT_TOLERANCE) && (heightPosition < targetHeight + HEIGHT_TOLERANCE);
    }

    public boolean hasLiftReachedHeight(String height) {
        if (height == LIFT_POLE_LOW) {
            return (inHeightTolerance(getPosition(), LIFT_POSITION_LOWPOLE));
        } else if (height == LIFT_POLE_GROUND) {
            return inHeightTolerance(getPosition(), LIFT_POSITION_GROUND);
        } else if (height == LIFT_POLE_HIGH) {
            return inHeightTolerance(getPosition(), LIFT_POSITION_HIGHPOLE);
        } else if (height == LIFT_POLE_MEDIUM) {
            return inHeightTolerance(getPosition(), LIFT_POSITION_MIDPOLE);
        } else {
            return false;
        }
    }

    public ArrayList<Double> getLiftMotorPowers() {
        ArrayList<Double> liftMotorPowers = new ArrayList<>();
        for (DcMotor liftMotor : liftMotors) {
            liftMotorPowers.add(liftMotor.getPower());
        }
        return liftMotorPowers;
    }

    public ArrayList<Integer> getLiftTargetPositions(){
        ArrayList<Integer> liftTargetPositions = new ArrayList<>();
        for(DcMotor liftMotor : liftMotors){
            liftTargetPositions.add(liftMotor.getTargetPosition());
        }
        return liftTargetPositions;
    }

    public ArrayList<Integer> getLiftPositions(){
        ArrayList<Integer> liftPositions = new ArrayList<>();
        for(DcMotor liftMotor : liftMotors){
            liftPositions.add(liftMotor.getCurrentPosition());
        }
        return liftPositions;
    }

    public boolean isLiftUp() {

        return (getPosition() > LIFT_POSITION_GROUND);
    }

    public void lowerWithGravity(){

    }

    public void resetEncoders() {
        for (DcMotor liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


}