package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Extension;
import org.firstinspires.ftc.teamcode.robot.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.robot.subsystems.LEDLights;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

import java.util.Map;
//import java.util.Map;

public class BrainSTEMRobot {

    public enum ConeCycleState {
        CONE_CYCLE_DOWN(false), CONE_CYCLE_UP(false), CONE_CYCLE_OFF(true);

        private boolean enabled;

        ConeCycleState(boolean enabled) {
            this.enabled = enabled;
        }

        public boolean isEnabled() {
            return this.enabled;
        }

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
    public LEDLights lights;

    public Grabber grabber;
    private Map stateMap;
    Constants constants = new Constants();

    private boolean isAuto;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, Map stateMap, boolean isAuto) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;
        this.isAuto = isAuto;
        // instantiate components turret, lift, arm, grabber
        turret = new Turret(hwMap, telemetry, stateMap, isAuto);
        lift = new Lift(hwMap, telemetry, stateMap);
        arm = new Extension(hwMap, telemetry, stateMap, isAuto);
        drive = new SampleMecanumDrive(hwMap);
        grabber = new Grabber(hwMap, telemetry, stateMap, isAuto);
        lights = new LEDLights(hwMap, telemetry, isAuto);


        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.LIFT_INTEGRAL_SUM, "0.0");
        stateMap.put(constants.LIFT_COMPLETE_TIME, "0");
        stateMap.put(lift.LIFT_TARGET_HEIGHT, lift.LIFT_POLE_HIGH);


        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void updateSystems() {
        stateMap.put(constants.SYSTEM_TIME, System.currentTimeMillis());

        lift.setState();
        grabber.setState(lift);
        turret.setState(lift);
        arm.setState((String) stateMap.get(arm.SYSTEM_NAME), lift);

    }

    public void coneCycle() {
        if (startliftDown()) {
            stateMap.put(constants.CONE_CYCLE_START_TIME, String.valueOf(System.currentTimeMillis()));
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
            telemetry.addData("Cone Cycle Loop", "startliftDown");
            telemetry.update();
        } else if (startGrabberAction()) {
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_IN_PROGRESS);
            telemetry.addData("Cone Cycle Loop", "startGrabberAction");
            telemetry.update();
        } else if (startLiftUp()) {
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_IN_PROGRESS);
            if (lift.getAvgLiftPosition() > 400) {
                grabber.maxOpen();
            }
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
            telemetry.addData("Cone Cycle Loop", "startLiftUp");
            telemetry.update();
        } else if (isConeCycleComplete()) {
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
            stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
            telemetry.addData("Cone Cycle Loop", "isConeCycleComplete");
            telemetry.update();
        }

        setConeCycleSystems();
    }

    private void setConeCycleSystems() {
        lift.setState();
        grabber.setState(lift);
    }

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

