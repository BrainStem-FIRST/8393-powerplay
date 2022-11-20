package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.util.StickyButton;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

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


    private String LIFT_HEIGHT = "POLE_HIGH";

    private final String GAMEPAD_1_RIGHT_TRIGGER_STATE = "GAMEPAD_1_RIGHT_TRIGGER_STATE";
    private final String GAMEPAD_1_RIGHT_TRIGGER_PRESSED = "GAMEPAD_1_RIGHT_TRIGGER_PRESSED";
    private final String MANUAL_DRIVE_MODE = "MANUAL";
    private final String AUTO_DRIVE_MODE = "AUTO";
    private final String DRIVE_MODE = "DRIVE";

    private boolean leftTriggerPressed = false;
    private boolean d2LeftTriggerPressed = false;
    private boolean d2RightTriggerPressed = false;


    private ToggleButton coneCycleToggleG = new ToggleButton();
    private StickyButton coneCyleStickyG = new StickyButton();
    private boolean coneCycleToggleGBoolean = false;
    private boolean coneCycleStickyBoolean = false;



    private StickyButton extensionFineAdjustUp = new StickyButton();
    private StickyButton extensionFineAdjustDown = new StickyButton();
    private StickyButton liftFineAdjustUp = new StickyButton();
    private  StickyButton liftFineAdjustDown = new StickyButton();




    private final double SLOWMODE  = 0.45;


    private boolean isDriverDriving = true;
    private boolean slowMode = false;

    private int liftDownIncrement;

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
        put(GAMEPAD_1_RIGHT_TRIGGER_PRESSED, false);
        put(GAMEPAD_1_RIGHT_TRIGGER_STATE, false);

    }};

    public void runOpMode() {

        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);

        SampleMecanumDrive driveCancelable = new SampleMecanumDrive(hardwareMap);
        driveCancelable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE);

        robot.arm.extendHome();
        robot.grabber.grabber.setPosition(0);

        waitForStart();

        while (!isStopRequested()) {
            setButtons();

            if (gamepad1.a || gamepad1.right_trigger > 0.5) {
                stateMap.put(constants.LIFT_START_TIME, String.valueOf(System.currentTimeMillis()));
                stateMap.put(constants.LIFT_INTEGRAL_SUM, "0.0");
            }

            if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                slowMode = true;
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(robot.lift.LIFT_TARGET_HEIGHT));
            } else {
                slowMode = false;
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
            }

//            if (toggleMap.get(GAMEPAD_1_Y_STATE)) {
//                stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
//            } else {
//                stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
//            }

            if (gamepad2.dpad_left) {
                stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
                stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
            } else if (gamepad2.dpad_up) {
                stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
            } else if (gamepad2.dpad_right) {
                stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
                stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
            }

            if (toggleMap.get(GAMEPAD_1_LEFT_TRIGGER_STATE)) {
                robot.grabber.open();
            }

            if(gamepad1.right_trigger > 0.5) {
                toggleMap.put(GAMEPAD_1_LEFT_TRIGGER_STATE, false);
                if(!((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
                    stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
                    stateMap.put(constants.CONE_CYCLE_START_TIME, String.valueOf(System.currentTimeMillis()));
                }
            }


            if (gamepad1.dpad_up) {
                isDriverDriving = false;
                stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
                toggleMap.put(GAMEPAD_1_A_STATE, true);
                Trajectory forwardTrajectory = driveCancelable.trajectoryBuilder(driveCancelable.getPoseEstimate())
                        .forward(4)
                        .addDisplacementMarker(() -> {
                            // This marker runs 2 inches into the trajectory
                            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_TARGET_HEIGHT);
                            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
                        })
                        .forward(38)
                        .addDisplacementMarker(() -> {
                            // This marker runs 2 inches into the trajectory
                            stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE);
                        })


                                .build();
                driveCancelable.followTrajectoryAsync(forwardTrajectory);



            } else if (gamepad1.dpad_down) {
                stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
                isDriverDriving = false;
                toggleMap.put(GAMEPAD_1_A_STATE, false);
                Trajectory reverseTrajectory = driveCancelable.trajectoryBuilder(driveCancelable.getPoseEstimate())
                        .back(25)
                        .addDisplacementMarker(() -> {
                            // This marker runs 2 inches into the trajectory
                            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);


                        })
                        .back(22)
                        .addDisplacementMarker(() -> {
                            // This marker runs 2 inches into the trajectory
                            stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE);
                        })

                        .build();
                driveCancelable.followTrajectoryAsync(reverseTrajectory);
            }  else if (gamepad1.dpad_right){
                    Trajectory strafeRightTrajectory = driveCancelable.trajectoryBuilder(driveCancelable.getPoseEstimate())
                            .strafeRight(3)
                            .build();
                    driveCancelable.followTrajectory(strafeRightTrajectory);



                } else if(gamepad1.dpad_left) {
                Trajectory strafeLeftTrajectory = driveCancelable.trajectoryBuilder(driveCancelable.getPoseEstimate())
                        .strafeLeft(3)
                        .build();
                driveCancelable.followTrajectory(strafeLeftTrajectory);


                if (stateMap.get(DRIVE_MODE).equals(MANUAL_DRIVE_MODE)) {

                    if (((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.right_stick_x != 0)) && !isDriverDriving) {

                        // driveCancelable.breakFollowing();

                        driveCancelable.setWeightedDrivePower(

                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );


                    } else {

                        if (slowMode) {

                            driveCancelable.setWeightedDrivePower(
                                    new Pose2d(
                                            (-gamepad1.left_stick_y) * 0.5,
                                            (-gamepad1.left_stick_x) * 0.5,
                                            (-gamepad1.right_stick_x) * 0.4
                                    )
                            );
                        } else {
                            driveCancelable.setWeightedDrivePower(
                                    new Pose2d(
                                            -gamepad1.left_stick_y,
                                            -gamepad1.left_stick_x,
                                            -gamepad1.right_stick_x
                                    )
                            );
                        }
                    }
                }
            }

            // Driver 2 //

            if (gamepad2.a){
                stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_LOW);
            }

            if (gamepad2.b){
                stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_MEDIUM);
            }

            if (gamepad2.y){
                stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_HIGH);
            }

            //Change extension preset values

            extensionFineAdjustUp.update(gamepad2.left_bumper);
            if (extensionFineAdjustUp.getState()){
                robot.arm.EXTENSION_POSITION_MAX += 20;
                robot.arm.extension.setPwmRange(new PwmControl.PwmRange(robot.arm.EXTENSION_POSITION_HOME, robot.arm.EXTENSION_POSITION_MAX));
            }

            extensionFineAdjustDown.update(gamepad2.right_bumper);
            if (extensionFineAdjustDown.getState()){
                robot.arm.EXTENSION_POSITION_MAX -= 20;
                robot.arm.extension.setPwmRange(new PwmControl.PwmRange(robot.arm.EXTENSION_POSITION_HOME, robot.arm.EXTENSION_POSITION_MAX));
            }

            // Change highpole preset value
            if (gamepad2.left_trigger > 0.2){
                d2LeftTriggerPressed = true;
            } else if (gamepad2.left_trigger < 0.2){
                d2LeftTriggerPressed = false;
            }

            liftFineAdjustUp.update(d2LeftTriggerPressed);
            if (liftFineAdjustUp.getState()){
                if (robot.lift.LIFT_POSITION_HIGHPOLE == 730){

                } else {
                    robot.lift.LIFT_POSITION_HIGHPOLE += 30;
                }

            }

            liftFineAdjustDown.update(d2RightTriggerPressed);
            if (liftFineAdjustDown.getState()){
                if (robot.lift.LIFT_POSITION_HIGHPOLE == 0){

                } else {
                    robot.lift.LIFT_POSITION_HIGHPOLE -= 30;
                }
            }

            driveCancelable.update();

            robot.updateSystems();

//            telemetry.addData("toggleMap", toggleMap);

            telemetry.update();
        }
    }

    private void setButtons() {
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
        toggleButton(GAMEPAD_1_B_STATE, GAMEPAD_1_B_IS_PRESSED, gamepad1.b);
        toggleButton(GAMEPAD_1_X_STATE, GAMEPAD_1_X_IS_PRESSED, gamepad1.x);
        toggleButton(GAMEPAD_1_RIGHT_TRIGGER_STATE, GAMEPAD_1_RIGHT_TRIGGER_PRESSED, gamepad1.right_trigger > 0.5);
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
