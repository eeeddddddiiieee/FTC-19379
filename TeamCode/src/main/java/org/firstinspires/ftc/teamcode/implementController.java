package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class implementController extends LinearOpMode {
    public void runOpMode(){

    }
    public void runImplementController(hwMecanum robot){
        //team element claw
        robot.teamElementArm.setPower(gamepad2.left_stick_y);
        robot.carousel.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

    }
}
