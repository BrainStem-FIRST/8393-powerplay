package org.firstinspires.ftc.teamcode.autonomous.selectionPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.autonomous.PoleFakeTroubleAuto;

@Autonomous(name="4 - Left Fake Pole Trouble")
@Disabled
public class LeftFakeTrouble extends PoleFakeTroubleAuto {
    public LeftFakeTrouble() {super(AutoOrientation.LEFT);}
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}