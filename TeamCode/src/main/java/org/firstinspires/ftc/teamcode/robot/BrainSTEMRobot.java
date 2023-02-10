package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.Extension;
import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.Flippers;
import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.Grabber;
//import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.Guide;
import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.LEDLights;
import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.teleopSubsystems.Turret;

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
    public Flippers flippers;
    //public PoleAligner poleAligner;
    public SampleMecanumDrive drive;
    public LEDLights lights;

//    public Guide guide;
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
//        guide = new Guide(hwMap, telemetry, stateMap);
        turret = new Turret(hwMap, telemetry, stateMap, isAuto);
        lift = new Lift(hwMap, telemetry, stateMap, isAuto);
        arm = new Extension(hwMap, telemetry, stateMap, isAuto);
        flippers = new Flippers(hwMap, telemetry, stateMap);
        //poleAligner = new PoleAligner(hwMap, telemetry, stateMap, isAuto);
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
//        stateMap.put(guide.SYSTEM_NAME, guide.UP_POSITION);


        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void updateSystems() {
        stateMap.put(constants.SYSTEM_TIME, System.currentTimeMillis());

        lift.setState();
        flippers.setState();
        grabber.setState(lift);
        turret.setState(lift);
        arm.setState((String) stateMap.get(arm.SYSTEM_NAME), lift);

    }
}

