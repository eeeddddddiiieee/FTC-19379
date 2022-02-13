package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hwMecanum;

public class driveControls {
    double x1;
    double y1;
    double yaw;
    double shift;
    double reverse;
    public driveControls(){
        shift=1;
    }
    public void runOpMode(){

    }
    public void driveController(hwMecanum robot, Gamepad gamepad1){
        /*
        y1 = -gamepad1.left_stick_y;
        x1 = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;




        if (gamepad1.left_bumper){
            reverse = -1;
        }
        else {
            reverse = 1;
        }

        double lb = Range.clip((((y1 * reverse) + yaw - (x1 * reverse)) * shift), -1.0, 1.0);
        double rb = Range.clip((((y1 * reverse) - yaw + (x1 * reverse)) * shift), -1.0, 1.0);
        double lf = Range.clip((((y1 * reverse) + yaw + (x1 * reverse)) * shift), -1.0, 1.0);
        double rf = Range.clip((((y1 * reverse) - yaw - (x1 * reverse)) * reverse * shift), -1.0, 1.0);

        robot.q1.setPower(rf);
        robot.q2.setPower(lf);
        robot.q3.setPower(lb);
        robot.q4.setPower(rb);
        */
        if (gamepad1.right_bumper) {
            shift = 0.35;
        }
        else {
            shift = 1;
        }
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y*shift,
                        -gamepad1.left_stick_x*shift,
                        -gamepad1.right_stick_x*shift
                )
        );
    }
}
