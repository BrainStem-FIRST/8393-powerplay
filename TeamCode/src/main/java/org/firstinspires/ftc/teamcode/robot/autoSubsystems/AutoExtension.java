package org.firstinspires.ftc.teamcode.robot.autoSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;


public class AutoExtension implements Subsystem {
    private Telemetry telemetry;

    // Three servos (plus the turret) work together to place cone to desired location
    public ServoImplEx extension;

    static final double MM_TO_INCHES = 0.0393700787;
    static final double MINIMUM_CLEARANCE_DISTANCE = 95.875 * MM_TO_INCHES;
    public final int LIFT_MIN_HEIGHT_TO_MOVE_EXTENSION = 75;


    // Servo Positions

    public final double EXTENSION_POSITION_HOME = 1900;
    public double EXTENSION_POSITION_MAX = 2372;

    public double EXTENSION_EDITABLE_POSITION = 0.4;
    // extension statemap values
    public final String SYSTEM_NAME = "EXTENSION"; //statemap key
    public final String DEFAULT_VALUE = "RETRACTED";
    public final String FULL_EXTEND = "EXTENDED";

    public final String LEFT_SIDE_UNCONTESTED_EXTENDED_AUTO = "LEFT_SIDE_UNCONTESTED_EXTENDED_AUTO";
    public final String RIGHT_SIDE_UNCONTESTED_EXTENDED_AUTO = "RIGHT_SIDE_UNCONTESTED_EXTENDED_AUTO";



    public final String AUTO_EXTENSION_DEPOSIT = "AUTO_EXTENSION_DEPOSIT";
    public final String LEFT_SIDE_EXTENDED_AUTO = "LEFT_SIDE_EXTENDED_AUTO";
    public final String RIGHT_SIDE_EXTENDED_AUTO = "RIGHT_SIDE_EXTENDED_AUTO";
    public final String AUTO_EXTENSION_DEPOSIT_PRELOAD = "AUTO_EXTENSION_DEPOSIT_PRELOAD";
    public final String AUTO_EXTENSION_COLLECT_RIGHT = "AUTO_EXTEND_COLLECT_RIGHT";
    public final String AUTO_EXTENSION_COLLECT_LEFT = "AUTO_EXTEND_COLLECT_LEFT";

    public final String AUTO_EXTENSION_LOW_COLLECT = "AUTO_EXTENSION_LOW_COLLECT";
    public final String DEFAULT_EXTEND_AUTO = "FULL_EXTEND_AUTO";
    public final String DEFAULT_SIDE_EXTENDED_AUTO = "DEFAULT_SIDE_EXTENDED_AUTO";
    public final String FOURTH_CYCLE = "FOURTH_CYCLE";

    public final String I_HATE_PHYSICS = "I_HATE_PHYSICS";
    public final String LAST_TWO_CYCLES = "LAST_TWO_CYCLES";
    public final String DEFAULT_SIDE_EXTENDED_AUTO_PRELOAD = "DEFAULT_SIDE_EXTEND_PRELOAD_AUTO";
    public final String SIDE_DEPOSIT = "SIDE_DEPOSIT";
    public final String TRANSITION_STATE = "TRANSITION";
    public final String DEFAULT_COLLECTING_VALUE = "DEFAULT_SIDE_COLLECTING_VALUE";
    Constants constants = new Constants();

    private Map stateMap;

    private boolean isAuto;

    public AutoExtension(HardwareMap hwMap, Telemetry telemetry, Map stateMap, boolean isAuto) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.isAuto = isAuto;

        extension = new CachingServo(hwMap.get(ServoImplEx.class, "extension"));

        extension.setPwmRange(new PwmControl.PwmRange(EXTENSION_POSITION_HOME, EXTENSION_POSITION_MAX));

        // Scale the operating range of Servos and set initial position
        extendHome();


    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    /************************* EXTENSION ARM UTILITIES **************************/

    // This method is intended for Teleop mode getting speed value coming from controller (-1..1)
    // Negative speed values will retract the extension arm.
    public void extend(double speed) {
        double currentPosition = extension.getPosition();

        //scale speed value so the extension moves in increments of 10% of the range at max speed
        double targetPosition = Range.clip(currentPosition + speed * 0.10, 0, 1);
        extension.setPosition(targetPosition / EXTENSION_POSITION_MAX);
        //Send telemetry message for debugging purposes
    }


    // Pulls the extension arm to its starting position (it is NOT in clear)
    public void extendHome() {
        extension.setPosition(0.025);
    }

    // Extends the arm to its maximum reach
    public void extendMax() {
        extension.setPosition(EXTENSION_EDITABLE_POSITION);
    }

    public void extendToTarget() {
        extension.setPosition(EXTENSION_EDITABLE_POSITION);
    }

    public void extendInAuto(double pos) {
        extension.setPosition(pos);
    }

    public void setState(String desiredState, AutoLift lift) {
        selectTransition(desiredState);
    }

    private void selectTransition(String desiredLevel) {
        switch (desiredLevel) {
            case DEFAULT_COLLECTING_VALUE: {
                extendInAuto(0.16);
                break;
            }
            case DEFAULT_VALUE: {
                extendHome();
                telemetry.addData("Home", "true");
                break;
            }
            case LAST_TWO_CYCLES: {
                extendInAuto(0.62);
                break;
            }
            case FULL_EXTEND: {
                extendToTarget();
                telemetry.addData("Full extend", "true");
                break;
            }
            case AUTO_EXTENSION_DEPOSIT: {
                extendInAuto(0.6);
                telemetry.addData("0.72", "true");
                break;
            }
            case AUTO_EXTENSION_DEPOSIT_PRELOAD: {
                extendInAuto(0.77);
                telemetry.addData("0.72", "true");
                break;
            }
            case AUTO_EXTENSION_COLLECT_RIGHT: {
                extendInAuto(0.64);
                telemetry.addData("0.8", "true");
                break;
            }
            case AUTO_EXTENSION_COLLECT_LEFT: {
                extendInAuto(0.64);
                telemetry.addData("0.8", "true");
                break;
            }
            case AUTO_EXTENSION_LOW_COLLECT: {
                extendInAuto(0.6);
                telemetry.addData("0.6", "true");
                break;
            }
            case DEFAULT_EXTEND_AUTO: {
                extendInAuto(0.3);
                telemetry.addData("0.3", "true");
                break;
            }
            case LEFT_SIDE_EXTENDED_AUTO: {

                extendInAuto(0.67);

                telemetry.addData("Side extended", "true");
                break;
            }
            case LEFT_SIDE_UNCONTESTED_EXTENDED_AUTO: {
                extendInAuto(0.74);
                break;
            }
            case RIGHT_SIDE_UNCONTESTED_EXTENDED_AUTO: {
                extendInAuto(0.74);
                break;
            }

            case RIGHT_SIDE_EXTENDED_AUTO: {

                extendInAuto(0.71);

                telemetry.addData("Side extended", "true");
                break;
            }
            case DEFAULT_SIDE_EXTENDED_AUTO_PRELOAD: {

                extendInAuto(0.37);

                telemetry.addData("Side extended", "true");
                break;
            }
            case FOURTH_CYCLE: {
                extendInAuto(0.45);
                break;
            }
            case I_HATE_PHYSICS: {
                extendInAuto(0.2);
                break;
            }
        }
    }

    public double getExtensionPosition() {
        return extension.getPosition();
    }

    public boolean isLiftTooLow(AutoLift lift) {
        return lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_EXTENSION;
    }

}