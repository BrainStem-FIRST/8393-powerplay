package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: TeleOp", group="Robot")
public class RobotTeleOp extends LinearOpMode {
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";
    private final String GAMEPAD_1_B_STATE = "GAMEPAD_1_B_STATE";
    private final String GAMEPAD_1_B_IS_PRESSED = "GAMEPAD_1_B_IS_PRESSED";
    private final String GAMEPAD_1_X_STATE = "GAMEPAD_1_X_STATE";
    private final String GAMEPAD_1_X_IS_PRESSED = "GAMEPAD_1_X_IS_PRESSED";
    private final String GAMEPAD_1_RIGHT_STICK_PRESSED = "GAMEPAD_1_RIGHT_STICK_PRESSED ";
    private final String GAMEPAD_1_RIGHT_STICK_STATE = "GAMEPAD_1_RIGHT_STICK";
    private final String GAMEPAD_1_LEFT_STICK_PRESSED = "GAMEPAD_1_LEFT_STICK_PRESSED";
    private final String GAMEPAD_1_LEFT_STICK_STATE =  "GAMEPAD_1_LEFT_STICK_STATE";
    private final String GAMEPAD_1_LEFT_TRIGGER_STATE  = "GAMEPAD_1_LEFT_TRIGGER_STATE";
    private final String GAMEPAD_1_LEFT_TRIGGER_PRESSED = "GAMEPAD_1_LEFT_TRIGGER_PRESSED";
    private final String GAMEPAD_1_Y_STATE = "GAMEPAD_1_Y_STATE";
    private final String GAMEPAD_1_Y_PRESSED = "GAMEPAD_1_Y_IS_PRESSED";

    private boolean leftTriggerPressed = false;
    private final double SLOWMODE  = 0.45;

    Constants constants = new Constants();



    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
        put(GAMEPAD_1_A_STATE, false);
        put(GAMEPAD_1_A_IS_PRESSED, false);
        put(GAMEPAD_1_B_STATE, false);
        put(GAMEPAD_1_B_IS_PRESSED, false);
        put(GAMEPAD_1_X_STATE, false);
        put(GAMEPAD_1_X_IS_PRESSED, false);
        put(GAMEPAD_1_RIGHT_STICK_STATE, false);
        put(GAMEPAD_1_RIGHT_STICK_PRESSED, false);
        put(GAMEPAD_1_LEFT_STICK_PRESSED, false);
        put(GAMEPAD_1_LEFT_TRIGGER_STATE ,false);
        put(GAMEPAD_1_LEFT_TRIGGER_PRESSED, false);
        put(GAMEPAD_1_Y_STATE, false);
        put(GAMEPAD_1_Y_PRESSED, false);
    }};

    public void runOpMode() {

        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
        SampleMecanumDriveCancelable driveCancelable = new SampleMecanumDriveCancelable(hardwareMap);
        driveCancelable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        waitForStart();

        while (!isStopRequested()) {
            setButtons();

            if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
            } else {
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
            }

            if (toggleMap.get(GAMEPAD_1_Y_STATE)) {
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

            if(gamepad1.right_trigger > 0.5){
                if(!((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
                    stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
                }
            }

            if (gamepad1.right_bumper) {
                Trajectory forwardTrajectory = driveCancelable.trajectoryBuilder(driveCancelable.getPoseEstimate())
                        .forward(40)
                        .build();
                driveCancelable.followTrajectoryAsync(forwardTrajectory);
            } else if (gamepad1.left_bumper) {
                Trajectory reverseTrajectory = driveCancelable.trajectoryBuilder(driveCancelable.getPoseEstimate())
                        .back(40)
                        .build();
                driveCancelable.followTrajectoryAsync(reverseTrajectory);
            } else {
                driveCancelable.setWeightedDrivePower(

                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            driveCancelable.update();

            robot.updateSystems();

            telemetry.addData("toggleMap", toggleMap);

            telemetry.addData("liftEncoders", robot.lift.getPosition());
            telemetry.update();
        }
    }

    private void setButtons() {
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
        toggleButton(GAMEPAD_1_B_STATE, GAMEPAD_1_B_IS_PRESSED, gamepad1.b);
        toggleButton(GAMEPAD_1_X_STATE, GAMEPAD_1_X_IS_PRESSED, gamepad1.x);
//        toggleButton(GAMEPAD_1_RIGHT_STICK_STATE, GAMEPAD_1_RIGHT_STICK_PRESSED, gamepad1.right_stick_button);
//        toggleButton(GAMEPAD_1_LEFT_STICK_STATE, GAMEPAD_1_LEFT_STICK_PRESSED, gamepad1.left_stick_button);
        toggleButton(GAMEPAD_1_LEFT_TRIGGER_STATE, GAMEPAD_1_LEFT_STICK_PRESSED,gamepad1.left_trigger >= 0.5);
        toggleButton(GAMEPAD_1_Y_STATE, GAMEPAD_1_Y_PRESSED, gamepad1.y);
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
