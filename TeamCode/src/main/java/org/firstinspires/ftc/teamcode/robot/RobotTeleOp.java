package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.NewToggleButton;
import org.firstinspires.ftc.teamcode.util.StickyButton;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "0 - TeleOp")
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
    private final String GAMEPAD_1_LEFT_STICK_STATE = "GAMEPAD_1_LEFT_STICK_STATE";
    private final String GAMEPAD_1_LEFT_TRIGGER_STATE = "GAMEPAD_1_LEFT_TRIGGER_STATE";
    private final String GAMEPAD_1_LEFT_TRIGGER_PRESSED = "GAMEPAD_1_LEFT_TRIGGER_PRESSED";
    private final String GAMEPAD_2_X_BUTTON_TOGGLE = "GAMEPAD_2_X_BUTTON_TOGGLE";
    private final String GAMEPAD_2_DPAD_DOWN_STATE = "GAMEPAD_2_DPAD_DOWN_STATE";
    private final String GAMEPAD_2_DPAD_DOWN_PRESSED = "GAMEPAD_2_DPAD_DOWN_PRESSED";
    private final String GAMEPAD_1_Y_STATE = "GAMEPAD_1_Y_STATE";
    private final String GAMEPAD_1_Y_PRESSED = "GAMEPAD_1_Y_IS_PRESSED";
    private final double AUTO_EXTENSION_ADJUSTMENT = 0.0;


    private String LIFT_HEIGHT = "POLE_HIGH";
    private String TURRET_POS = "CENTER_POSITION";
    private String EXTENSION_POS = "DEFAULT_VALUE";

    private final String GAMEPAD_1_RIGHT_TRIGGER_STATE = "GAMEPAD_1_RIGHT_TRIGGER_STATE";
    private final String GAMEPAD_1_RIGHT_TRIGGER_PRESSED = "GAMEPAD_1_RIGHT_TRIGGER_PRESSED";

    private final String GAMEPAD_2_RIGHT_TRIGGER_STATE = "GAMEPAD_2_RIGHT_TRIGGER_STATE";

    private final String GAMEPAD_2_X_BUTTON_PRESSED = "GAMEPAD_1_RIGHT_TRIGGER_PRESSED";

    private boolean leftTriggerPressed = false;
    private boolean d2LeftTriggerPressed = false;
    private boolean d2RightTriggerPressed = false;

    private boolean coneCycleCenterAdjust = false;

    private boolean extensionCenter = false;

    private ToggleButton coneCycleToggleG = new ToggleButton();
    private StickyButton coneCyleStickyG = new StickyButton();
    private boolean coneCycleToggleGBoolean = false;
    private boolean coneCycleStickyBoolean = false;


    private StickyButton extensionFineAdjustUp = new StickyButton();
    private StickyButton extensionFineAdjustDown = new StickyButton();
    private StickyButton liftFineAdjustUp = new StickyButton();
    private StickyButton liftFineAdjustDown = new StickyButton();


    private Pose2d zeroPose = new Pose2d(0, 0, Math.toRadians(0));


    boolean disableDrivetrain = false;


    private boolean isDriverDriving = true;
    private boolean slowMode = false;


    private boolean resetComplete = true;

    //lift bring in delay
    private boolean bringLiftDownBoolean = false;
    private ElapsedTime liftDelay = new ElapsedTime();
    private ElapsedTime resetDelay = new ElapsedTime();

    private boolean liftDelayCollectingBoolean = false;
    private ElapsedTime liftDelayCollecting = new ElapsedTime();

    private int bottomAdjustmentHeight = 0;
    private double driver2_ground_adjusted_subheight = 0;

    private double conePickUp = 0.31;
    private double driver2_placement_adjusted_subheight;

    private NewToggleButton leftFlipperToggleButton = new NewToggleButton();
    private NewToggleButton rightFlipperToggleButton = new NewToggleButton();

    private boolean gamepad2RightTriggerPressed = false;
    private boolean gamepad2LeftTriggerPressed = false;

    Constants constants = new Constants();

    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
        put(GAMEPAD_2_RIGHT_TRIGGER_STATE, false);
        put(GAMEPAD_1_A_STATE, false);
        put(GAMEPAD_1_A_IS_PRESSED, false);
        put(GAMEPAD_1_B_STATE, false);
        put(GAMEPAD_1_B_IS_PRESSED, false);
        put(GAMEPAD_1_X_STATE, false);
        put(GAMEPAD_1_X_IS_PRESSED, false);
        put(GAMEPAD_1_RIGHT_STICK_STATE, false);
        put(GAMEPAD_1_RIGHT_STICK_PRESSED, false);
        put(GAMEPAD_1_LEFT_STICK_PRESSED, false);
        put(GAMEPAD_1_LEFT_TRIGGER_STATE, false);
        put(GAMEPAD_1_LEFT_TRIGGER_PRESSED, false);
        put(GAMEPAD_1_Y_STATE, false);
        put(GAMEPAD_1_Y_PRESSED, false);
        put(GAMEPAD_1_RIGHT_TRIGGER_STATE, false);
        put(GAMEPAD_2_X_BUTTON_TOGGLE, false);
        put(GAMEPAD_2_X_BUTTON_PRESSED, false);
        put(GAMEPAD_2_RIGHT_TRIGGER_STATE, false);
        put(GAMEPAD_2_DPAD_DOWN_STATE, false);
        put(GAMEPAD_2_DPAD_DOWN_PRESSED, false);

    }};

    public RobotTeleOp() {

    }

    public RobotTeleOp(boolean disableDrivetrain) {
        this.disableDrivetrain = disableDrivetrain;
    }

    public void runOpMode() {

        HashMap<String, String> stateMap = new HashMap<String, String>() {{
        }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap, false);

        SampleMecanumDrive driveCancelable = new SampleMecanumDrive(hardwareMap);
        driveCancelable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(constants.EXTENSION_TARGET, String.valueOf(1));
        stateMap.put(robot.flippers.LEFT_SYSTEM_NAME, robot.flippers.LEFT_UP);
        stateMap.put(robot.flippers.RIGHT_SYSTEM_NAME, robot.flippers.RIGHT_UP);
        stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);

        while (!opModeIsActive()) {
            stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
            robot.updateSystems();

        }


        waitForStart();


        while (!isStopRequested()) {

            telemetry.addData("Stack Increment", robot.lift.stackIncrement);
            if (gamepad2.right_trigger > 0.5 && !gamepad2RightTriggerPressed) {
                robot.lift.incrementStack();
                gamepad2RightTriggerPressed = true;
            } else if (gamepad2.right_trigger <= 0.5) {
                gamepad2RightTriggerPressed = false;
            }

            if (gamepad2.left_trigger > 0.5 && !gamepad2LeftTriggerPressed) {
                robot.lift.decrementStack();
                gamepad2LeftTriggerPressed = true;
            } else if (gamepad2.left_trigger <= 0.5) {
                gamepad2LeftTriggerPressed = false;
            }

            if(toggleMap.get(GAMEPAD_2_DPAD_DOWN_STATE)) {
                conePickUp = 0.09;
                robot.grabber.cap = true;
            } else {
                conePickUp = 0.31;
                robot.grabber.cap = false;
            }

            telemetry.addData("ConePickup", conePickUp);

            if ((gamepad2.right_stick_button && gamepad2.left_stick_button) || (gamepad1.right_stick_button && gamepad1.left_stick_button)) {
                resetComplete = false;
                resetDelay.startTime();
            }

            if (resetComplete == false && resetDelay.seconds() < 0.4) {
                robot.lift.setAllMotorPowers(-0.15);
                robot.turret.centerTurret();
                robot.arm.extendHome();
            } else if (resetDelay.seconds() < 0.5 && resetComplete == false) {
                robot.lift.resetEncoders();
            } else {
                setButtons();

                resetComplete = true;
                resetDelay.reset();

                if (stateMap.get(robot.lift.LIFT_SYSTEM_NAME) != robot.lift.LIFT_POLE_GROUND) {
                    if (gamepad1.right_trigger > 0.05 && gamepad1.right_trigger < 0.9) {
                        robot.lift.setSubheight(gamepad1.right_trigger);
                    } else if (gamepad1.right_trigger >= 0.9) {
                        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
                        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                        robot.arm.extendHome();
                        robot.grabber.open();
                        liftDelay.reset();
                        bringLiftDownBoolean = true;
                    } else {
                        robot.lift.setSubheight(0);
                    }
                } else if (gamepad1.right_trigger > 0.5 && robot.lift.getAvgLiftPosition() < 500) {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                    liftDelay.reset();
                    liftDelayCollecting.reset();
                    liftDelayCollectingBoolean = true;
                }

                if (liftDelayCollectingBoolean && liftDelayCollecting.seconds() > 0.05) {
                    liftDelayCollectingBoolean = false;
                    if (driver2_ground_adjusted_subheight < 0.1) {
                        robot.lift.setSubheight(conePickUp);
                    } else {
                        driver2_ground_adjusted_subheight += 0.5;
                    }
                }

                if(bringLiftDownBoolean) {
                    if (liftDelay.seconds() > 0.05 && liftDelay.seconds() <= 0.2) {
                        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                    }

                    if (liftDelay.seconds() > 0.2) {
                        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
                    }

                    if (liftDelay.seconds() > 0.15 && robot.lift.getAvgLiftPosition() > 750) {
                        toggleMap.put(GAMEPAD_1_A_STATE, false);
                    }

                    if (liftDelay.seconds() > 0.25) {
                        toggleMap.put(GAMEPAD_1_A_STATE, false);
                        bringLiftDownBoolean = false;
                    }
                }

                if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                    slowMode = true;
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(robot.lift.LIFT_TARGET_HEIGHT));
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.TELE_EXTENSION);
                    if(bringLiftDownBoolean) {
                    } else {
                        stateMap.put(robot.arm.SYSTEM_NAME, EXTENSION_POS);
                        stateMap.put(robot.turret.SYSTEM_NAME, TURRET_POS);

                    }
                } else {
                    slowMode = false;
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                    robot.turret.centerTurret();
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
                }
                if(gamepad1.a && !stateMap.get(robot.lift.LIFT_SYSTEM_NAME).equals(robot.lift.LIFT_POLE_GROUND)){
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                }

                if ((gamepad1.a && stateMap.get(robot.lift.LIFT_SYSTEM_NAME).equals(robot.lift.LIFT_POLE_GROUND)) ||
                        gamepad2.y ||
                        gamepad2.b ||
                        gamepad2.a) {
                    driver2_placement_adjusted_subheight = 0;
                } else if (gamepad1.a) {
                    driver2_ground_adjusted_subheight = 0;
                }

                if (gamepad2.dpad_left) {
                    TURRET_POS = robot.turret.LEFT_POSITION;
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
                    extensionCenter = false;
                } else if (gamepad2.dpad_up) {
                    robot.turret.centerTurret();
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                    TURRET_POS = robot.turret.CENTER_POSITION;
                    EXTENSION_POS = robot.arm.DEFAULT_VALUE;
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                    extensionCenter = true;
                } else if (gamepad2.dpad_right) {
                    TURRET_POS = robot.turret.RIGHT_POSITION;
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
                    extensionCenter = false;
                } else if (gamepad1.left_trigger > 0.5) {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
                    robot.lift.setSubheight(0);
                }


                //////////////
                // Driver 2 //
                //////////////

                if (driver2_ground_adjusted_subheight != 0) {
                    robot.lift.setSubheight(driver2_ground_adjusted_subheight);
                }

                if (gamepad2.a) {
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_LOW);
                }

                if (gamepad2.b) {
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_MEDIUM);
                }

                if (gamepad2.y) {
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_HIGH);
                }

                if(gamepad1.dpad_up){
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                }

                if(gamepad1.dpad_down){
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                }

                if (robot.lift.getPosition() < 100 || gamepad2.dpad_up || extensionCenter) {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                    extensionCenter = true;
                } else {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.TELE_EXTENSION);
                    extensionCenter = false;
                }


                if (gamepad2.left_bumper && robot.lift.getPosition() > 600) {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.TELE_EXTENSION);
                    if ((robot.arm.EXTENSION_EDITABLE_POSITION > 0.9)) {
                        robot.arm.extension.setPosition(0.9);
                    } else {
                        robot.arm.EXTENSION_EDITABLE_POSITION += 0.027;
                        robot.arm.extension.setPosition(robot.arm.EXTENSION_EDITABLE_POSITION);
                        robot.arm.extendToTarget();
                    }

                }

                if (gamepad2.right_bumper && robot.lift.getPosition() > 600) {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.TELE_EXTENSION);
                    if ((robot.arm.EXTENSION_EDITABLE_POSITION < 0.15)) {
                        robot.arm.extension.setPosition(0.15);
                    } else {
                        robot.arm.EXTENSION_EDITABLE_POSITION -= 0.027;
                        robot.arm.extension.setPosition(robot.arm.EXTENSION_EDITABLE_POSITION);

                    }

                }

                if(!disableDrivetrain) {
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
                                        -gamepad1.right_stick_x * 0.75
                                )
                        );
                    }
                }
                leftFlipperToggleButton.update(gamepad1.left_bumper);
                rightFlipperToggleButton.update(gamepad1.right_bumper);

                if(leftFlipperToggleButton.getState()) {
                    stateMap.put(robot.flippers.LEFT_SYSTEM_NAME, robot.flippers.LEFT_DOWN);
                } else {
                    stateMap.put(robot.flippers.LEFT_SYSTEM_NAME, robot.flippers.LEFT_UP);
                }

                if(rightFlipperToggleButton.getState()) {
                    stateMap.put(robot.flippers.RIGHT_SYSTEM_NAME, robot.flippers.RIGHT_DOWN);
                } else {
                    stateMap.put(robot.flippers.RIGHT_SYSTEM_NAME, robot.flippers.RIGHT_UP);
                }








                telemetry.addData("Extension PWM Position: ", robot.arm.EXTENSION_EDITABLE_POSITION);
                telemetry.addData("Extension State: ", stateMap.get(robot.arm.SYSTEM_NAME));
                telemetry.addData("Grabber State: ", stateMap.get(robot.grabber.SYSTEM_NAME));
                telemetry.addData("Extension Last Position: ", robot.arm.getExtensionLastValue());
                telemetry.addData("Lift Positions: ", robot.lift.getLiftPositions());
                telemetry.addData("Lift powers: ", robot.lift.getLiftMotorPowers());
                telemetry.addData("Bottom adjustment height: ", robot.lift.LIFT_POSITION_GROUND);
                telemetry.addData("Gamepad 1 A Button Toggle State: ", toggleMap.get(GAMEPAD_1_A_STATE));
                telemetry.addData("Lift Position Being Set: ", stateMap.get(robot.lift.LIFT_TARGET_HEIGHT));
                telemetry.addData("Gamepad 2 Right Trigger State: ", toggleMap.get(GAMEPAD_2_RIGHT_TRIGGER_STATE));
                telemetry.addData("Cone Up Adjust: ", conePickUp);



                driveCancelable.update();

                robot.updateSystems();

                telemetry.update();

            }
        }
    }

    private void setButtons() {
        toggleButton(GAMEPAD_2_DPAD_DOWN_STATE, GAMEPAD_2_DPAD_DOWN_PRESSED, gamepad2.dpad_down);
        toggleButton(GAMEPAD_2_X_BUTTON_TOGGLE, GAMEPAD_2_X_BUTTON_PRESSED, gamepad2.x);
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
        toggleButton(GAMEPAD_1_B_STATE, GAMEPAD_1_B_IS_PRESSED, gamepad1.b);
        toggleButton(GAMEPAD_1_X_STATE, GAMEPAD_1_X_IS_PRESSED, gamepad1.x);
        toggleButton(GAMEPAD_1_RIGHT_TRIGGER_STATE, GAMEPAD_1_RIGHT_TRIGGER_PRESSED, gamepad1.right_trigger > 0.5);
        toggleButton(GAMEPAD_1_LEFT_TRIGGER_STATE, GAMEPAD_1_LEFT_STICK_PRESSED, gamepad1.left_trigger >= 0.5);
        toggleButton(GAMEPAD_1_Y_STATE, GAMEPAD_1_Y_PRESSED, gamepad1.y);

    }

    private boolean toggleButton(String buttonStateName, String buttonPressName,
                                 boolean buttonState) {

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
