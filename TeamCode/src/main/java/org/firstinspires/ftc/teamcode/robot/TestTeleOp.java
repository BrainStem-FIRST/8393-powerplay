package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test TeleOp", group = "Robot")
public class TestTeleOp extends LinearOpMode {
    public void runOpMode() {
        Extension extension = new Extension(hardwareMap, telemetry, null);
        Lift lift = new Lift(hardwareMap, telemetry, null);

        while(!opModeIsActive()){}

        while(opModeIsActive()){

            telemetry.addData("Lift Positions: ", lift.getLiftPositions());
            telemetry.update();

            /*if(gamepad1.a){
                extension.extendHome();
            }
            if(gamepad1.b){
                extension.extendMax();
            }*/
        }

    }
}
