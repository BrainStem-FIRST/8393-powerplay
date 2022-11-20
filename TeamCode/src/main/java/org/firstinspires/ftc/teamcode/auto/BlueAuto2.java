package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="2 Blue Auto")
public class BlueAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Auto2 auto = new Auto2(Auto2.AllianceColor.BLUE, this);
        auto.run();

    }

}