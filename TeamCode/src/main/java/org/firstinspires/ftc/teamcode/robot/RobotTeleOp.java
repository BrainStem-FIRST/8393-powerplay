package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.StickyButton;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "TELEOPERATED", group = "Robot")
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
    private final String GAMEPAD_1_Y_STATE = "GAMEPAD_1_Y_STATE";
    private final String GAMEPAD_1_Y_PRESSED = "GAMEPAD_1_Y_IS_PRESSED";
    private final double AUTO_EXTENSION_ADJUSTMENT = 0.0;


    private String LIFT_HEIGHT = "POLE_HIGH";
    private String TURRET_POS = "CENTER_POSITION";
    private String EXTENSION_POS = "DEFAULT_VALUE";

    private final String GAMEPAD_1_RIGHT_TRIGGER_STATE = "GAMEPAD_1_RIGHT_TRIGGER_STATE";
    private final String GAMEPAD_1_RIGHT_TRIGGER_PRESSED = "GAMEPAD_1_RIGHT_TRIGGER_PRESSED";

    private final String GAMEPAD_2_RIGHT_TRIGGER_STATE = "GAMEPAD_2_RIGHT_TRIGGER_STATE";
    private final String GAMEPAD_2_RIGHT_TRIGGER_PRESSED = "GAMEPAD_2_RIGHT_TRIGGER_PRESSED";

    private final String GAMEPAD_2_X_BUTTON_PRESSED = "GAMEPAD_1_RIGHT_TRIGGER_PRESSED";

    private boolean leftTriggerPressed = false;
    private boolean d2LeftTriggerPressed = false;
    private boolean d2RightTriggerPressed = false;

    private boolean coneCycleCenterAdjust = false;


    private ToggleButton coneCycleToggleG = new ToggleButton();
    private ToggleButton gamepad2xbutton = new ToggleButton();
    private StickyButton coneCyleStickyG = new StickyButton();
    private boolean coneCycleToggleGBoolean = false;
    private boolean coneCycleStickyBoolean = false;


    private StickyButton extensionFineAdjustUp = new StickyButton();
    private StickyButton extensionFineAdjustDown = new StickyButton();
    private StickyButton liftFineAdjustUp = new StickyButton();
    private StickyButton liftFineAdjustDown = new StickyButton();
    private StickyButton bottomHeightStickyButtonRightTrigger = new StickyButton();
    private StickyButton bottomHeightStickyButtonLeftTrigger = new StickyButton();

    private Pose2d zeroPose = new Pose2d(0, 0, Math.toRadians(0));


    private final double SLOWMODE = 0.45;

    boolean disableDrivetrain = false;


    private boolean isDriverDriving = true;
    private boolean slowMode = false;

    private int liftDownIncrement;

    //open grabber boolean
    private boolean openGrabberConeCycle = false;

    //lift bring in delay
    private boolean bringLiftDownBoolean = false;
    private ElapsedTime liftDelay = new ElapsedTime();

    private boolean liftDelayCollectingBoolean = false;
    private ElapsedTime liftDelayCollecting = new ElapsedTime();

    private int bottomAdjustmentHeight = 0;

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
        put(GAMEPAD_1_LEFT_TRIGGER_STATE, false);
        put(GAMEPAD_1_LEFT_TRIGGER_PRESSED, false);
        put(GAMEPAD_1_Y_STATE, false);
        put(GAMEPAD_1_Y_PRESSED, false);
        put(GAMEPAD_1_RIGHT_TRIGGER_PRESSED, false);
        put(GAMEPAD_1_RIGHT_TRIGGER_STATE, false);
        put(GAMEPAD_2_X_BUTTON_TOGGLE, false);
        put(GAMEPAD_2_X_BUTTON_PRESSED, false);

    }};
    //DO NOT DELETE THIS OR COMMENT IT OUT! IT LOOKS UNUSED BUT IF DELETED, TELEOP WON'T RUN!
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
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        //stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(constants.EXTENSION_TARGET, String.valueOf(1));


        while (!opModeIsActive()) {


            telemetry.addData("Robot ::", "Init");
            telemetry.update();

            robot.lights.setBothLEDAmber();
            stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
            robot.updateSystems();

        }


        waitForStart();


        while (!isStopRequested()) {
            robot.lights.setBothLEDRed();

            bottomHeightStickyButtonRightTrigger.update(gamepad2.right_trigger > 0.5);
            bottomHeightStickyButtonLeftTrigger.update(gamepad2.left_trigger > 0.5);



            if ((gamepad2.right_stick_button && gamepad2.left_stick_button) || (gamepad1.right_stick_button && gamepad1.left_stick_button)) {
                robot.lift.setAllMotorPowers(-0.25);
                robot.lift.LIFT_POSITION_GROUND = 0;
                robot.turret.centerTurret();
                robot.arm.extendHome();
                robot.lift.resetEncoders();
            } else {
                setButtons();

                if (stateMap.get(robot.lift.LIFT_SYSTEM_NAME) != robot.lift.LIFT_POLE_GROUND) {
                    if (gamepad1.right_trigger > 0.05 && gamepad1.right_trigger < 0.9) {
                        robot.lift.setSubheight(gamepad1.right_trigger);
                    } else if (gamepad1.right_trigger >= 0.9) {
                        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                        robot.lift.LIFT_POSITION_GROUND = 0;
                        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                        robot.arm.extendHome();
                        liftDelay.reset();
                        robot.grabber.maxOpen();
                        bringLiftDownBoolean = true;
                    } else {
                        robot.lift.setSubheight(0);
                    }
                } else if (gamepad1.right_trigger > 0.5 && robot.lift.getAvgLiftPosition() < 300) {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                    liftDelay.reset();
                    liftDelayCollectingBoolean = true;
                    liftDelayCollecting.reset();
                    robot.lift.setSubheight(0.15);
                }
                if(bringLiftDownBoolean){
                    if(liftDelay.seconds() > 0.1){
                        toggleMap.put(GAMEPAD_1_A_STATE, false);
                        robot.lift.LIFT_POSITION_GROUND = 0;
                        bringLiftDownBoolean = false;
                    }
                }

                if(liftDelayCollectingBoolean) {
                    if(liftDelayCollecting.seconds() > 0.075) {
                        liftDelayCollectingBoolean = false;
                        if(robot.lift.LIFT_POSITION_GROUND == 0) {
                            robot.lift.LIFT_POSITION_GROUND = 50;
                        } else if (bottomAdjustmentHeight != 0) {
                            robot.lift.LIFT_POSITION_GROUND = 300;
                            updateBottomHeightAdjustment(false);
                        }
                    }
                }

                if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                    slowMode = true;
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(robot.lift.LIFT_TARGET_HEIGHT));
                    stateMap.put(robot.turret.SYSTEM_NAME, TURRET_POS);
                    if(bringLiftDownBoolean) {
                        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                    } else {
                        stateMap.put(robot.arm.SYSTEM_NAME, EXTENSION_POS);
                    }
                    robot.lift.LIFT_POSITION_GROUND = 0;
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

                if (gamepad2.dpad_left) {
                    TURRET_POS = robot.turret.LEFT_POSITION;
                    EXTENSION_POS = robot.arm.FULL_EXTEND;
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
                } else if (gamepad2.dpad_up) {
                    robot.turret.centerTurret();
                    robot.arm.extendHome();
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                    TURRET_POS = robot.turret.CENTER_POSITION;
                    EXTENSION_POS = robot.arm.DEFAULT_VALUE;
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                } else if (gamepad2.dpad_right) {
                    TURRET_POS = robot.turret.RIGHT_POSITION;
                    EXTENSION_POS = robot.arm.FULL_EXTEND;
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
                } else if (gamepad1.left_trigger > 0.5) {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_GROUND);
                    robot.lift.setSubheight(-0.15);
                    robot.lift.LIFT_POSITION_GROUND = 0;
                }

                // Driver 2 //

                if (gamepad2.a) {
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_LOW);
                }

                if (gamepad2.b) {
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_MEDIUM);
                }

                if (gamepad2.y) {
                    stateMap.put(robot.lift.LIFT_TARGET_HEIGHT, robot.lift.LIFT_POLE_HIGH);
                }

//                if(gamepad2.dpad_down) {
//                    stateMap.put(robot.guide.SYSTEM_NAME, robot.guide.DOWN_POSITION);
//                }
//
//                if(gamepad2.dpad_up) {
//                    stateMap.put(robot.guide.SYSTEM_NAME, robot.guide.UP_POSITION);
//                }


                //Change extension preset values

                //extensionFineAdjustUp.update(gamepad2.left_bumper);
                if (gamepad2.left_bumper) {
                    robot.arm.EXTENSION_EDITABLE_POSITION += 0.027;
                    robot.arm.extension.setPwmRange(new PwmControl.PwmRange(robot.arm.EXTENSION_POSITION_HOME, robot.arm.EXTENSION_POSITION_MAX));
                    robot.arm.extendToTarget();
                }

                //extensionFineAdjustDown.update(gamepad2.right_bumper);
                if (gamepad2.right_bumper) {
                    robot.arm.EXTENSION_EDITABLE_POSITION -= 0.027;
                    robot.arm.extension.setPwmRange(new PwmControl.PwmRange(robot.arm.EXTENSION_POSITION_HOME, robot.arm.EXTENSION_POSITION_MAX));
                    robot.arm.extendToTarget();
                }

                //double weightedDriveSpeedMultiplier  = robot.lift.getAvgLiftPosition() < 200 ? 0.7 : 0.5;
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


                if (bottomHeightStickyButtonRightTrigger.getState()) {
                    updateBottomHeightAdjustment(true);
                    robot.lift.LIFT_POSITION_GROUND = bottomAdjustmentHeight;
                }
                if (bottomHeightStickyButtonLeftTrigger.getState()) {
                    updateBottomHeightAdjustment(false);
                    robot.lift.LIFT_POSITION_GROUND = bottomAdjustmentHeight;
                }
                if (gamepad2.x) {
                    bottomAdjustmentHeight = 0;
                    robot.lift.LIFT_POSITION_GROUND = bottomAdjustmentHeight;
                }

                telemetry.addData("Grabber State", stateMap.get(robot.grabber.SYSTEM_NAME));
                telemetry.addData("Lift Positions: ", robot.lift.getLiftPositions());
                telemetry.addData("Lift powers: ", robot.lift.getLiftMotorPowers());
                telemetry.addData("Bottom adjustment height: ", robot.lift.LIFT_POSITION_GROUND);
                telemetry.addData("Gamepad 1 A Button Toggle State ", toggleMap.get(GAMEPAD_1_A_STATE));
                telemetry.addData("Lift Position Being Set ", stateMap.get(robot.lift.LIFT_TARGET_HEIGHT));


                driveCancelable.update();

                robot.updateSystems();

                telemetry.update();

            }
        }
    }
    private void updateBottomHeightAdjustment(boolean goingUp){
        if(goingUp) {
            if(bottomAdjustmentHeight == 0) {
                bottomAdjustmentHeight = 40;
                return;
            } else if (bottomAdjustmentHeight == 40) {
                bottomAdjustmentHeight = 82;
                return;
            } else if (bottomAdjustmentHeight == 82) {
                bottomAdjustmentHeight = 110;
                return;
            } else if (bottomAdjustmentHeight == 110) {
                bottomAdjustmentHeight = 140;
                return;
            } else if (bottomAdjustmentHeight == 140) {
                bottomAdjustmentHeight = 300;
                return;
            }
        } else {
            if(bottomAdjustmentHeight == 0) {
                bottomAdjustmentHeight = 0;
                return;
            } else if (bottomAdjustmentHeight == 40) {
                bottomAdjustmentHeight = 0;
                return;
            } else if (bottomAdjustmentHeight == 82) {
                bottomAdjustmentHeight = 40;
                return;
            } else if (bottomAdjustmentHeight == 110) {
                bottomAdjustmentHeight = 82;
                return;
            } else if (bottomAdjustmentHeight == 140) {
                bottomAdjustmentHeight = 110;
                return;
            } else if (bottomAdjustmentHeight == 300) {
                bottomAdjustmentHeight = 140;
                return;
            }
        }

    }

    private void setButtons() {
        toggleButton(GAMEPAD_2_X_BUTTON_TOGGLE, GAMEPAD_2_X_BUTTON_PRESSED, gamepad2.x);
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
        toggleButton(GAMEPAD_1_B_STATE, GAMEPAD_1_B_IS_PRESSED, gamepad1.b);
        toggleButton(GAMEPAD_1_X_STATE, GAMEPAD_1_X_IS_PRESSED, gamepad1.x);
        toggleButton(GAMEPAD_1_RIGHT_TRIGGER_STATE, GAMEPAD_1_RIGHT_TRIGGER_PRESSED, gamepad1.right_trigger > 0.5);
//        toggleButton(GAMEPAD_1_RIGHT_STICK_STATE, GAMEPAD_1_RIGHT_STICK_PRESSED, gamepad1.right_stick_button);
//        toggleButton(GAMEPAD_1_LEFT_STICK_STATE, GAMEPAD_1_LEFT_STICK_PRESSED, gamepad1.left_stick_button);
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