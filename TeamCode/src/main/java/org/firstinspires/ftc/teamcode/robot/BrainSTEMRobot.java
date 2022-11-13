package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotserver.internal.webserver.controlhubupdater.ChUpdaterCommManager;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Map;

public class BrainSTEMRobot {
    private static final class BrainSTEMRobotConstants {

    }

    public enum BrainSTEMRobotState {

    }

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor.RunMode currentDrivetrainMode;

    private Telemetry telemetry;
    private OpMode opMode;

    // declare robot components
    public Turret turret;
    public Lift lift;
    public Extension arm;
    public SampleMecanumDrive drive;
    public Grabber grabber;
    private Map stateMap;
    Constants constants = new Constants();

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret = new Turret(hwMap, telemetry, stateMap);
        lift = new Lift(hwMap, telemetry, stateMap);
        arm = new Extension(hwMap, telemetry);
        drive = new SampleMecanumDrive(hwMap);
        grabber = new Grabber(hwMap, telemetry, stateMap);

        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void updateSystems() {
        stateMap.put(constants.SYSTEM_TIME, System.currentTimeMillis());

        if (((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)) {
            coneCycle();
        } else {
            lift.setState();
            grabber.setState();
            turret.setState(lift);
            arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
        }

    }

//    public void coneCycle() {
//        telemetry.addData("coneCycle", true);
//        if (!lift.isLiftUp()) {
//            grabber.openGrabber();
//        }
//
//        stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
//        lift.setState();
//
//        if (!lift.isLiftUp()) {
//            grabber.closeGrabber();
//        } else {
//            grabber.openGrabber();
//        }
//
//        stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
//        lift.setState();
//
//        stateMap.put(constants.CONE_CYCLE, constants.STATE_COMPLETE);
//        telemetry.addData("coneCycle", false);
//    }

    public void coneCycle() {
        if (startliftDown()) {
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
        } else if (startGrabberAction()) {
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_IN_PROGRESS);
        } else if (startLiftUp()) {
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
        } else if (isConeCycleComplete()) {
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        }

        //setConeCycleSystems();
    }

    public void slowTurret(int targetPosition, String position) {
        double currentPosition;
        if (stateMap.get(turret.SYSTEM_NAME).equals(turret.LEFT_POSITION)) {
            currentPosition = 0.45;
        } else if (stateMap.get(turret.SYSTEM_NAME).equals(turret.RIGHT_POSITION)) {
            currentPosition = 0.9;
        } else {
            currentPosition = (0.9 + 0.45) / 2;
        }

        if (currentPosition < targetPosition) {
            while (currentPosition < targetPosition) {
                turret.transitionToPosition(currentPosition + 0.05, currentPosition + 0.05);
                currentPosition++;
            }
            stateMap.put(turret.SYSTEM_NAME, position);
        } else if (currentPosition > targetPosition) {
            while (currentPosition > targetPosition) {
                turret.transitionToPosition(currentPosition - 0.05, currentPosition - 0.05);
                currentPosition--;
            }
            stateMap.put(turret.SYSTEM_NAME, position);
        }
    }

    /*private void setConeCycleSystems() {
        lift.setState();
        grabber.setState(lift);
    }*/

    private boolean startLiftUp() {
        return ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_NOT_STARTED);
    }

    private boolean startliftDown() {
        return (((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) &&
                ((String) (stateMap.get(constants.CYCLE_LIFT_DOWN))).equalsIgnoreCase(constants.STATE_NOT_STARTED));
    }

    private boolean startGrabberAction() {
        return ((String) stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_NOT_STARTED);
    }

    private boolean isConeCycleComplete() {
        return (((String) stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_COMPLETE));
    }
}

