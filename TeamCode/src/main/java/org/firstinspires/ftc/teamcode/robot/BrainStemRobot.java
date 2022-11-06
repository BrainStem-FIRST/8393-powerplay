package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.HashMap;
import java.util.Map;
import java.util.Timer;

public class BrainStemRobot {

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

    public BrainStemRobot(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret  = new Turret(hwMap, telemetry);
        lift    = new Lift(hwMap, telemetry, stateMap);
        arm     = new Extension(hwMap, telemetry);
        drive   = new SampleMecanumDrive(hwMap);
        grabber   = new Grabber(hwMap, telemetry, stateMap);

        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void initializeRobotPosition(){
        lift.initializePosition();
        lift.moveToMinHeight();  // Raise lift to clear side panels. This does not clear the arm holding cone.
        // Extend the arm so it clears corner of the robot when swinging
        turret.initializePosition();
        lift.raiseHeightTo(0);
    }

    public void updateSystems() {
        telemetry.addData("robotStateMap" , stateMap);
        stateMap.put(constants.SYSTEM_TIME, System.currentTimeMillis());


        if(((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
            coneCycle();
        } else {
            lift.setState();
            turret.setState((String) stateMap.get(turret.SYSTEM_NAME), lift);
            arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
        }

    }

    public void coneCycle() {
        if(startliftDown()) {
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
        } else if(startGrabberAction()){
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_IN_PROGRESS);
        } else if(startLiftUp()){
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
        } else if(isConeCycleComplete()){
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        }

        setConeCycleSystems();
    }

    private void setConeCycleSystems() {
        lift.setState();
        grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME), lift);
    }

    private boolean startLiftUp() {
        return ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_NOT_STARTED);
    }

    private boolean startliftDown() {
        return (((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) &&
                ((String)(stateMap.get(constants.CYCLE_LIFT_DOWN))).equalsIgnoreCase(constants.STATE_NOT_STARTED));
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

