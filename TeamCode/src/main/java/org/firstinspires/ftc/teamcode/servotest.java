/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//parent dir
package org.firstinspires.ftc.teamcode;

//library imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;



@TeleOp(name="Mecanum: Teleop", group="hwMecanum")
//TODO: disable teleop by uncommenting the following line
//@Disabled

//teleop class
public class servotest extends LinearOpMode {


    //claw vars.
    double clawOffset = 0;
    final double CLAW_SPEED = 0.02;

    @Override
    public void runOpMode() {
        //new object robot

        //all vars used in the opmode

        /*
        TODO:

        */


        //turn on the hw map from the class (hwMecanum)
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {
            robot.arm1.setPosition(gamepad1.left_stick_x);
            robot.arm2.setPosition(gamepad1.left_stick_x);
            robot.claw.setPosition(gamepad1.right_trigger);
            telemetry.addData("arm1 position",robot.arm1.getPosition());
            telemetry.addData("arm2 position",robot.arm2.getPosition());
            telemetry.addData("claw position",robot.claw.getPosition());

            //sleep so other tasks can run
            sleep(50);
            if (isStopRequested()) return;
        }
    }
}

