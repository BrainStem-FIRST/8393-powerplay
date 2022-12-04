package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;


public class Extension {
    private Telemetry telemetry;

    // Three servos (plus the turret) work together to place cone to desired location
    public ServoImplEx extension;

    static final double MM_TO_INCHES = 0.0393700787;
    static final double MINIMUM_CLEARANCE_DISTANCE = 95.875 * MM_TO_INCHES;
    public final int LIFT_MIN_HEIGHT_TO_MOVE_EXTENSION = 75;


    // Servo Positions

    public final double EXTENSION_POSITION_HOME = 1800;    // Fully retracted
    public double EXTENSION_POSITION_MAX  = 2372;    // Fully extended

    public double EXTENSION_EDITABLE_POSITION = 0.75;
    // extension statemap values
    public final String SYSTEM_NAME = "EXTENSION"; //statemap key
    public final String DEFAULT_VALUE = "RETRACTED";
    public final String FULL_EXTEND = "EXTENDED";
    public final String AUTO_EXTENSION = "AUTO_EXTEND";
    public final String TRANSITION_STATE = "TRANSITION";
    Constants constants = new Constants();

    private Map stateMap;

    public Extension(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        extension = new CachingServo(hwMap.get(ServoImplEx.class, "extension"));

        extension.setPwmRange(new PwmControl.PwmRange(EXTENSION_POSITION_HOME, EXTENSION_POSITION_MAX));

        // Scale the operating range of Servos and set initial position
        extendHome();


    }

    /************************* EXTENSION ARM UTILITIES **************************/

    // This method is intended for Teleop mode getting speed value coming from controller (-1..1)
    // Negative speed values will retract the extension arm.

    public void extend(double speed) {
        double currentPosition = extension.getPosition();

        //scale speed value so the extension moves in increments of 10% of the range at max speed
        double targetPosition = Range.clip(currentPosition + speed*0.10, 0, 1);
        extension.setPosition(targetPosition/EXTENSION_POSITION_MAX);
        //Send telemetry message for debugging purposes
        telemetry.addData("Speed of move:","%.2f", speed);
        telemetry.addData("Extension Position:","%.2f", targetPosition);
        telemetry.update();
    }


    // Pulls the extension arm to its starting position (it is NOT in clear)
    public void extendHome() {
        telemetry.addData("Bringing in: ", "true");
        extension.setPosition(0.01);
        telemetry.addData("Position goal:", extension.getPosition());
    }

    // Extends the arm to its maximum reach
    public void extendMax() {
        extension.setPosition(EXTENSION_EDITABLE_POSITION);
    }

    public void extendToTarget() {
        extension.setPosition(EXTENSION_EDITABLE_POSITION);
    }

    public void extendInAuto(double pos){
        extension.setPosition(pos);
    }

    public void setState(String desiredState, Lift lift){
        if(isLiftTooLow(lift)){
            selectTransition((String) DEFAULT_VALUE);
        }
        else{
            selectTransition(desiredState);
        }

    }

    private void selectTransition(String desiredLevel){
        switch(desiredLevel) {
            case DEFAULT_VALUE: {
                extendHome();
                break;
            }
            case FULL_EXTEND: {
                extendToTarget();
                break;
            }
            case AUTO_EXTENSION: {
                extendInAuto(0.2);
                break;
            }
        }
    }

    public double getExtensionPosition() {
        return extension.getPosition();
    }

    public boolean isLiftTooLow(Lift lift) {
        return lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_EXTENSION;
    }

}