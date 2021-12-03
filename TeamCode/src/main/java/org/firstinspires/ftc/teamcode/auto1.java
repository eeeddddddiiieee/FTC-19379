package org.firstinspires.ftc.teamcode;
//library imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hwMecanum;
//import deez nuts



/*
This is the RETARD AUTO. At all expense, please do not use this for the final robot. Time based
park, carousel, idk
TODO: This auto does not use roadrunner or PID. This is a backup auto.
 */
public class auto1 extends LinearOpMode {
    hwMecanum robot;
    public static final double ticksPerInch=537.7/11.87373601358268;
    public void runOpMode() throws InterruptedException{
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        waitForStart();


        telemetry.update();
        //park in shipping depot
        //Comment this out if need to
        moveToPosition(.5,(int)(ticksPerInch*6));
        reorientIMU(90,-.5,.5,.5,1.5,.001,0);
        moveToPosition(.9,(int)(ticksPerInch*30));

        //
        //robot.claw.setPosition(robot.servoClosed);
        robot.arm.setPower(.5);
        sleep(500);
        robot.arm.setPower(0);


    }

    public void driveForward(double speed){
        Trajectory move1 = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(speed)
                .build();
        robot.followTrajectory(move1);
    }

    public void turn(double x,double y,double angle){
        Trajectory move2 = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(x,y),Math.toRadians(angle))
                .build();
        robot.followTrajectory(move2);
    }

    public void moveToPosition(double motorPower, int ticks){
        resetDriveEncoders();

        double motorVelocity = motorPower*2700;

        robot.q3.setTargetPosition(ticks);
        robot.q2.setTargetPosition(ticks);
        robot.q4.setTargetPosition(ticks);
        robot.q1.setTargetPosition(ticks);

        robot.q3.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.q2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.q4.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.q1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        robot.q1.setVelocity(motorVelocity);
        robot.q4.setVelocity(motorVelocity);
        robot.q3.setVelocity(motorVelocity);
        robot.q2.setVelocity(motorVelocity);

        while(robot.q3.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "not there yet");
            telemetry.addData("left back", robot.q3.getCurrentPosition());
            telemetry.addData("left front", robot.q2.getCurrentPosition());
            telemetry.addData("right back", robot.q4.getCurrentPosition());
            telemetry.addData("right front", robot.q1.getCurrentPosition());
            telemetry.addData("busy1", robot.q3.isBusy());
            telemetry.addData("busy2", robot.q2.isBusy());
            telemetry.addData("busy3", robot.q1.isBusy());
            telemetry.addData("busy4", robot.q4.isBusy());
            telemetry.addData("Target", robot.q3.getTargetPosition());
            telemetry.update();
        }

        stopDrivetrain();

    }
    public double getFirstAngle(){
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.RADIANS).firstAngle;
    }

    public double getSecondAngle(){
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.RADIANS).secondAngle;
    }

    public double getThirdAngle(){
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.RADIANS).thirdAngle;
    }
    //PID algorithim to set robot at specific heading
    public void reorientIMU(double targetAngle, double left, double right, double threshold, double kp, double ki, double kd) {
        //get the current value in radians
        double currentValue = getFirstAngle();
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        //initialize PID variables
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(targetAngle - currentValue) > threshold) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            robot.q2.setPower(output * left);
            robot.q3.setPower(output * left);
            robot.q1.setPower(output * right);
            robot.q4.setPower(output * right);

            //get the current value from the IMU
            currentValue = getFirstAngle();
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Left Power", robot.q3.getPower());
            telemetry.addData("Right Power", robot.q4.getPower());
            telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
        stopDrivetrain();
    }

    //strafe right or left for a specified amount of time
    public void strafe(double power, int sleepTime){
            resetDriveEncoders();

        useEncoders();

        robot.q3.setPower(-power);
        robot.q2.setPower(power);
        robot.q4.setPower(power);
        robot.q1.setPower(-power);

        sleep(sleepTime);

        robot.q3.setPower(0);
        robot.q2.setPower(0);
        robot.q4.setPower(0);
        robot.q1.setPower(0);
    }

    public void resetDriveEncoders(){
        robot.q2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.q3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.q1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.q4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    //kills the drive train motors
    public void stopDrivetrain(){
        robot.q3.setPower(0);
        robot.q2.setPower(0);
        robot.q1.setPower(0);
        robot.q4.setPower(0);
    }
    //run mode to use encoder
    public void useEncoders(){
        robot.q2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.q3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.q1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.q4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.q2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.q3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.q4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.q1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    //move using encoder for a specified amount of time
    public void moveWithEncoders(double motorPower, int sleepTime){
        useEncoders();

        robot.q2.setPower(motorPower);
        robot.q3.setPower(motorPower);
        robot.q4.setPower(motorPower);
        robot.q1.setPower(motorPower);

        sleep(sleepTime);

        robot.q2.setPower(0);
        robot.q3.setPower(0);
        robot.q4.setPower(0);
        robot.q1.setPower(0);
    }
    //strafe at the specified angle at specified time
    public void strafeAngle(double speed, double degrees, int sleepTime){
        //set runmode to RUN_USING_ENCODERS
        useEncoders();

        //convert angle to radians
        double radians = Math.toRadians(degrees);

        //subtract pi/4 because the rollers are angled pi/4 radians
        double robotAngle = radians - (Math.PI/4);
        double[] motorPower;
        motorPower = new double[4];

        //set motor powers based on the specified angle
        motorPower[0] = Math.cos(robotAngle);
        motorPower[1] = Math.sin(robotAngle);
        motorPower[2] = Math.sin(robotAngle);
        motorPower[3] = Math.cos(robotAngle);


        //because of limitations with the sin and cos functions, the motors are not always going at the speed that is specified
        //in order to do this, we multiply each motor power by the desired speed over the highest motor power
        double maxPower = 0;
        for (double power : motorPower) {
            if (Math.abs(power) > maxPower) {
                maxPower = Math.abs(power);
            }
        }

        double ratio;

        if (maxPower == 0) {
            ratio = 0;
        } else {
            ratio = speed / maxPower;
        }

        double leftFront = Range.clip((ratio * motorPower[0]), -1, 1);
        double rightFront = Range.clip((ratio * motorPower[1]), -1, 1);
        double leftBack = Range.clip((ratio * motorPower[2]) , -1, 1);
        double rightBack = Range.clip((ratio * motorPower[3]), -1, 1);

        //set motor powers
        robot.q2.setPower(leftFront);
        robot.q1.setPower(rightFront);
        robot.q3.setPower(leftBack);
        robot.q4.setPower(rightBack);

        sleep(sleepTime);

        stopDrivetrain();
    }
    //using PID algorithm to make sure it strafes in a straight line
    public void strafingPID(double motorPower, double sleepTime, double kp, double ki, double kd){
        double targetAngle = getFirstAngle();
        double targetTime = getRuntime()+(sleepTime/1000);


        robot.q3.setPower(-motorPower);
        robot.q2.setPower(motorPower);
        robot.q4.setPower(motorPower);
        robot.q1.setPower(-motorPower);

        double error = 0;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;
        double outputChange;

        //measuring the error and integral and sets the motor power based on the PID
        while ((getRuntime()<targetTime)){
            error = targetAngle-getFirstAngle();
            integral += error;
            derivative = error-lastError;
            outputChange = (error*kp)+(integral*ki)+(derivative*kd);


            robot.q3.setPower(-motorPower-outputChange);
            robot.q2.setPower(motorPower-outputChange);
            robot.q4.setPower(motorPower+outputChange);
            robot.q1.setPower(-motorPower+outputChange);

            lastError = error;

        }

        stopDrivetrain();
    }
    public void wrapIMU(double targetAngle, double left, double right, double threshold, double kp, double ki, double kd) {
        //get the current value in radians
        double currentValue = getFirstAngleWrapped();
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        //initialize PID variables
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(targetAngle - currentValue) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            robot.q2.setPower(output * left);
            robot.q3.setPower(output * left);
            robot.q1.setPower(output * right);
            robot.q4.setPower(output * right);

            //get the current value from the IMU
            currentValue = getFirstAngleWrapped()
            ;
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Left Power", robot.q3.getPower());
            telemetry.addData("Right Power", robot.q4.getPower());
            telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
        stopDrivetrain();
    }
    public double getFirstAngleWrapped(){
        if (getFirstAngle() < 0) {
            return getFirstAngle() + (Math.PI*2);
        }
        else{
            return getFirstAngle();
        }
    }
}
