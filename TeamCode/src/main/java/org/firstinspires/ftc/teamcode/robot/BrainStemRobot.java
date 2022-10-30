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

    public final String STATE_IN_PROGRESS = "IN PROGRESS";
    public final String STATE_COMPLETE = "COMPLETE";
    public final String CONE_CYCLE = "CONE CYCLE";
    private final String STATE_NOT_STARTED = "NOT STARTED";
    public final String CYCLE_LIFT_DOWN = STATE_NOT_STARTED;
    public final String CYCLE_LIFT_UP = STATE_NOT_STARTED;
    public final String CYCLE_GRABBER = STATE_NOT_STARTED;




    public BrainStemRobot(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret  = new Turret(hwMap, telemetry);
        lift    = new Lift(hwMap, telemetry, stateMap);
        arm     = new Extension(hwMap, telemetry);
        drive   = new SampleMecanumDrive(hwMap);
        grabber   = new Grabber(hwMap, telemetry);

        stateMap.put(CONE_CYCLE, STATE_COMPLETE);

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

        if(((String)stateMap.get(CONE_CYCLE)).equalsIgnoreCase(STATE_IN_PROGRESS)){
            coneCycle();
        } else {
            lift.setState();
            grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME));
            turret.setState((String) stateMap.get(turret.SYSTEM_NAME), lift);
            arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
        }

    }

    public void coneCycle() {
        String grabberDesiredState =  null;
        if (lift.isCollectionHeight()) {
            grabberDesiredState = grabber.CLOSED_STATE;
        } else {
            grabberDesiredState = grabber.OPEN_STATE;
        }

        if(isConeCycleComplete()){
            stateMap.put(CYCLE_LIFT_DOWN, STATE_NOT_STARTED);
            stateMap.put(CYCLE_GRABBER, STATE_NOT_STARTED);
            stateMap.put(CYCLE_LIFT_UP,STATE_NOT_STARTED);
        }

        if(startliftDown()) {
            stateMap.put(CYCLE_LIFT_DOWN,STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
        } else if(startGrabberAction()){
            stateMap.put(CYCLE_GRABBER, STATE_IN_PROGRESS);

        } else if(startLiftUp()){
            stateMap.put(CYCLE_LIFT_UP, STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
        }

        if(((String) stateMap.get(CYCLE_LIFT_DOWN)).equalsIgnoreCase(STATE_COMPLETE) && ((String)stateMap.get(CYCLE_GRABBER)).equalsIgnoreCase(STATE_COMPLETE)){
            stateMap.put(CYCLE_LIFT_UP,STATE_IN_PROGRESS);
            lift.setState();
            stateMap.put(CYCLE_LIFT_UP, STATE_COMPLETE);
        }

        setConeCycleSystems();
    }

    private void setConeCycleSystems() {
        lift.setState();
        grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME));
    }

    private boolean startLiftUp() {
        return ((String) stateMap.get(CYCLE_GRABBER)).equalsIgnoreCase(STATE_COMPLETE) &&
                !((String) stateMap.get(CYCLE_LIFT_UP)).equalsIgnoreCase(STATE_IN_PROGRESS);
    }

    private boolean startliftDown() {
        return (!((String) stateMap.get(CYCLE_GRABBER)).equalsIgnoreCase(STATE_IN_PROGRESS));
    }

    private boolean startGrabberAction() {
        return ((String) stateMap.get(CYCLE_LIFT_DOWN)).equalsIgnoreCase(STATE_COMPLETE) &&
                !((String) stateMap.get(CYCLE_GRABBER)).equalsIgnoreCase(STATE_IN_PROGRESS);
    }

    private boolean isConeCycleComplete() {
        return (((String) stateMap.get(CYCLE_LIFT_DOWN)).equalsIgnoreCase(STATE_COMPLETE) &&
                ((String) stateMap.get(CYCLE_GRABBER)).equalsIgnoreCase(STATE_COMPLETE) &&
                ((String) stateMap.get(CYCLE_LIFT_UP)).equalsIgnoreCase(STATE_COMPLETE));
    }
}

