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
public class mecanumTeleOp extends LinearOpMode {


    //claw vars.
    double clawOffset = 0;
    final double CLAW_SPEED = 0.02;

    @Override
    public void runOpMode() {
        //new object robot

        //all vars used in the opmode
        double x1;
        double y1;
        double x2;
        double y2;
        double yaw;

        /*
        TODO:

        */
        double anglecorrection = -Math.PI / 4;
        double cos45 = Math.cos(anglecorrection);
        double sin45 = Math.sin(anglecorrection);

        //turn on the hw map from the class (hwMecanum)
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        telemetry.update();
        robot.q1.setDirection(DcMotor.Direction.FORWARD);
        robot.q2.setDirection(DcMotor.Direction.FORWARD);
        robot.q3.setDirection(DcMotor.Direction.FORWARD);
        robot.q4.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        double shift=1;
        double reverse = 1;
        while (opModeIsActive()) {
            //set power to left stick x and y axis
            y1 = -gamepad1.left_stick_y;
            x1 = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;


            if (gamepad1.right_bumper) {
                shift = 0.35;
            }
            else {
                shift = 1;
            }

                double lb = Range.clip((((y1 * reverse) + yaw - (x1 * reverse)) * shift), -1.0, 1.0);
                double rb = Range.clip((((y1 * reverse) - yaw + (x1 * reverse)) * shift), -1.0, 1.0);
                double lf = Range.clip((((y1 * reverse) + yaw + (x1 * reverse)) * shift), -1.0, 1.0);
                double rf = Range.clip((((y1 * reverse) - yaw - (x1 * reverse)) * reverse * shift), -1.0, 1.0);
            /*
            TODO:

            */
                y2 = y1 * cos45 + x1 * sin45;
                x2 = x1 * cos45 - y1 * sin45;

                //cardinal direction control

                robot.q1.setPower(-rf);
                robot.q2.setPower(lf);
                robot.q3.setPower(lb);
                robot.q4.setPower(-rb);
                /*
                robot.q2.setPower(gamepad1.left_stick_y);
                robot.q1.setPower(gamepad1.right_stick_y);
                robot.q3.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
                robot.q4.setPower(gamepad2.right_stick_y);*/
            /*
            robot.q1.setPower(yaw);
            robot.q2.setPower(yaw);
            robot.q3.setPower(yaw);
            robot.q4.setPower(yaw);
            */


                if (gamepad2.right_bumper)
                {clawOffset += CLAW_SPEED;}
                else if (gamepad2.left_bumper)
                {clawOffset -= CLAW_SPEED;}

                clawOffset = Range.clip(clawOffset, -0.5, 0.5);
                robot.claw.setPosition(hwMecanum.MID_SERVO + clawOffset);


                //arm code
                double armPower = gamepad2.right_trigger-gamepad2.left_trigger;
                robot.arm.setPower(armPower * .15);
                //robot.claw.setPosition(gamepad2.right_trigger);
            /*
            if (gamepad2.y) {
                robot.arm.setPower(hwMecanum.ARM_UP_POWER);
            } else if (gamepad2.a)
                robot.arm.setPower(hwMecanum.ARM_DOWN_POWER);
            else
                robot.arm.setPower(0.0);
            */


                //lift code
                double liftPower = gamepad2.left_stick_y;
                robot.lift.setPower(liftPower * .20);

                //code for the drive and carousel.
                //if i hold down x, then I use the triggers to run the carousel
                //otherwise if x is not held down, it drives the middle wheels
                double middleDrive = gamepad1.left_trigger - gamepad1.right_trigger;
                if (gamepad1.x) {
                    robot.carousel.setPower(middleDrive);
                } else {
                    robot.drive1.setPower(middleDrive);
                }

                if (gamepad1.b) {
                    robot.carousel.setPower(-.75);
                    sleep(300);
                    robot.carousel.setPower(-1);
                    sleep(700);
                    robot.carousel.setPower(0);
                }
                if (gamepad1.y) {
                    robot.carousel.setPower(.75);
                    sleep(300);
                    robot.carousel.setPower(1);
                    sleep(700);
                    robot.carousel.setPower(0);
                }


                //update telemetry for the phone TODO: USE FOR TROUBLESHOOTING
                //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            /*telemetry.addData("lift",  "%.2f", robot. lift);
            telemetry.addData("arm", "%.2f", robot.arm);
            telemetry.addData("leftf",  "%.2f", robot. q1);
            telemetry.addData("rightf", "%.2f", robot.q2);
            telemetry.addData("leftb",  "%.2f", robot. q3);
            telemetry.addData("rightb", "%.2f", robot.q4);
            telemetry.update();
            */


                //sleep so other tasks can run
                sleep(50);
                if (isStopRequested()) return;
            }
        }
    }

