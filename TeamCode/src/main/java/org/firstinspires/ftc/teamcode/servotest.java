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





//teleop class
@TeleOp(name="servotest", group="hwMecanum")

public class servotest extends LinearOpMode {


    //claw vars.
    double clawOffset = 0;
    final double claw_SPEED = 0.004;
    double TEclawOffset = 0;
    final double TEclaw_SPEED = 0.02;

    double TE1clawOffset = 0;
    final double TE1claw_SPEED = 0.02;


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
            if (gamepad1.a){
                robot.arm1.setPosition(hwMecanum.low);
                robot.arm2.setPosition(1.02-hwMecanum.low);
            }
            if (gamepad1.b){
                robot.arm1.setPosition(hwMecanum.high);
                robot.arm2.setPosition(1.02-hwMecanum.high);
            }
            if (gamepad1.back){
                robot.arm1.setPosition(hwMecanum.high1);
                robot.arm2.setPosition(1.02-hwMecanum.high1);
            }

            if (gamepad1.right_bumper)
            {clawOffset += claw_SPEED;}
            else if (gamepad1.left_bumper)
            {clawOffset=0;

            }

            clawOffset = Range.clip(clawOffset, 0, .03);
            robot.claw.setPosition(hwMecanum.OPEN_CLAW + clawOffset);

            if (gamepad2.right_bumper)
            {TEclawOffset += TEclaw_SPEED;}
            else if (gamepad2.left_bumper)
            {TEclawOffset -= TEclaw_SPEED;}

            TEclawOffset = Range.clip(TEclawOffset, -0.5, 0.5);
            robot.teamElementServo.setPosition(.5 + TEclawOffset);

            //Team element arm code
            if (gamepad2.a)
            {TE1clawOffset += TE1claw_SPEED;}
            else if (gamepad2.x)
            {TE1clawOffset -= TE1claw_SPEED;}

            TE1clawOffset = Range.clip(TE1clawOffset, -0.5, 0.5);
            robot.teamElementArm.setPosition(.5 + TE1clawOffset);

            telemetry.addData("arm1 position",robot.teamElementArm.getPosition());
            telemetry.addData("arm2 position",robot.teamElementServo.getPosition());
            telemetry.addData("claw position",robot.claw.getPosition());
            telemetry.update();

            //sleep so other tasks can run
            sleep(50);
            if (isStopRequested()) return;
        }
    }
}

