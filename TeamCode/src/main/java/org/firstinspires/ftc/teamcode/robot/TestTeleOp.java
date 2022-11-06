package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: Test Scaffold", group="Robot")
public class TestTeleOp extends LinearOpMode {
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";
    private final String GAMEPAD_1_B_STATE = "GAMEPAD_1_B_STATE";
    private final String GAMEPAD_1_B_IS_PRESSED = "GAMEPAD_1_B_IS_PRESSED";
    private final String GAMEPAD_1_X_STATE = "GAMEPAD_1_X_STATE";
    private final String GAMEPAD_1_X_IS_PRESSED = "GAMEPAD_1_X_IS_PRESSED";
    Constants constants = new Constants();



    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
        put(GAMEPAD_1_A_STATE, false);
        put(GAMEPAD_1_A_IS_PRESSED, false);
        put(GAMEPAD_1_B_STATE, false);
        put(GAMEPAD_1_B_IS_PRESSED, false);
        put(GAMEPAD_1_X_STATE, false);
        put(GAMEPAD_1_X_IS_PRESSED, false);
    }};

    public void runOpMode() {

        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
//        robot.initializeRobotPosition();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        waitForStart();
      while (opModeIsActive()) {

        setButtons();

        if (toggleMap.get(GAMEPAD_1_A_STATE)) {
            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
        } else {
            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        }

        if (toggleMap.get(GAMEPAD_1_B_STATE)) {
            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
        } else {
            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        }


        if (gamepad1.dpad_left) {
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
        } else if (gamepad1.dpad_up) {
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        } else if (gamepad1.dpad_right) {
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
        }
        if(gamepad1.right_trigger > 0.5 && stateMap.get(constants.CONE_CYCLE).equalsIgnoreCase(constants.STATE_NOT_STARTED)){
            stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -Math.pow(gamepad1.left_stick_y, 2),
                        -Math.pow(gamepad1.left_stick_x, 2),
                        -Math.pow(gamepad1.right_stick_x, 2)
                )
        );

        drive.update();

        robot.updateSystems();

        telemetry.addData("toggleMap", toggleMap);

        telemetry.update();
        }
    }

    private void setButtons() {
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
        toggleButton(GAMEPAD_1_B_STATE, GAMEPAD_1_B_IS_PRESSED, gamepad1.b);
        toggleButton(GAMEPAD_1_X_STATE, GAMEPAD_1_X_IS_PRESSED, gamepad1.x);
    }

    private boolean toggleButton(String buttonStateName, String buttonPressName, boolean buttonState) {
        boolean buttonPressed = toggleMap.get(buttonPressName);
        boolean toggle = toggleMap.get(buttonStateName);

        if (buttonState) {
            if (!buttonPressed) {
                toggleMap.put(buttonStateName, !toggle);
                toggleMap.put(buttonPressName, true);
            }
        } else {
            toggleMap.put(buttonPressName, false);
        }

        return toggleMap.get(buttonStateName);
    }

}
