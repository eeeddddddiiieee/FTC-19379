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

//parent folder
package org.firstinspires.ftc.teamcode;

//import libraries
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import java.util.List;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


//define class
@Config
public class hwMecanum extends MecanumDrive {


    //hardware declaration
    public DcMotorEx q1, q2, q3, q4; //names of drive motors
    public DcMotorEx lift; //lift motor
    public DcMotorEx arm; //arm motor
    public Servo claw; //claw servo
    public Servo arm1; //claw servo
    public Servo arm2;
    public DcMotorEx intake;
    public DcMotorEx carousel;
    HardwareMap hwMap = null; //hardware map
    private List<DcMotorEx> motors; //drive motor list from l49
    public BNO055IMU imu; //imu
    private VoltageSensor batteryVoltageSensor; //batt volt sensor
    private Pose2d lastPoseOnTurn; //for servo position

    //trajectory shit idk
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectoryFollower follower;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
    public static double LATERAL_MULTIPLIER=1;
    public static double VX_WEIGHT=1;
    public static double VY_WEIGHT=1;
    public static double OMEGA_WEIGHT=1;

    //servo and arm constants
    public static final double MID_SERVO=0.5;
    public static final double ARM_UP_POWER=0.45;
    public static final double ARM_DOWN_POWER=-0.45;
    public static final double servoClosed=.69;
    public static final double servoOpen=.8;
    //timer
    private ElapsedTime period = new ElapsedTime();

    //constructor
    public hwMecanum(HardwareMap ahwMap){
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap; //init hw map for following devices
        //telemetry.update();
        follower=new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(.5,.5,Math.toRadians(5.0)),.5);
        LynxModuleUtil.ensureMinimumFirmwareVersion(hwMap);
        batteryVoltageSensor=hwMap.voltageSensor.iterator().next();

        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //imu init
        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        //remap imu axes because ours is mounted vertically
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XZY, AxesSigns.PNP);

        //hardware init
        q2=hwMap.get(DcMotorEx.class, "left_drivef"); //    left drive front init
        q1=hwMap.get(DcMotorEx.class, "right_drivef"); //right drive front init
        q3=hwMap.get(DcMotorEx.class, "left_driveb"); //left drive back init
        q4=hwMap.get(DcMotorEx.class, "right_driveb"); //right drive back init
        intake=hwMap.get(DcMotorEx.class, "intake");
        arm=hwMap.get(DcMotorEx.class, "arm"); //arm init
        lift=hwMap.get(DcMotorEx.class, "teamelement"); //lift init
        claw=hwMap.get(Servo.class,"claw"); // claw init
        carousel=hwMap.get(DcMotorEx.class, "carousel");


        motors=Arrays.asList(q2,q3,q4,q1);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }



        //direction corrections for motor orientations TODO: TWEAK IF HAVING DRIVING ISSUES
        q1.setDirection(DcMotor.Direction.REVERSE);
        q2.setDirection(DcMotor.Direction.FORWARD);
        q3.setDirection(DcMotor.Direction.FORWARD);
        q4.setDirection(DcMotor.Direction.REVERSE);

        q1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        q2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        q3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        q4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        //set all power to 0
        q1.setPower(0);
        q2.setPower(0);
        q3.setPower(0);
        q4.setPower(0);
        arm.setPower(0);
        drive1.setPower(0);
        lift.setPower(0);
        carousel.setPower(0);
        claw.setPosition(.69); //servo is coded off of position, not power. (NOT CONTINUOUS)

        //init encoders (for auto)
        q1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        q2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        q3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        q4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //only drive motors use encoders retard
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //init claw and set position TODO: tweak based on robot starting config
        //claw = hwMap.get(ServoEx.class, "claw");
        //claw.setPosition(MID_SERVO);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

    }
    public TrajectoryBuilder trajectoryBuilder (Pose2d startPose){
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder (Pose2d startPose,boolean reversed){
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder (Pose2d startPose,double startHeading){
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder (Pose2d startPose){
        return new TrajectorySequenceBuilder(startPose,VEL_CONSTRAINT, ACCEL_CONSTRAINT,MAX_ANG_VEL, MAX_ANG_ACCEL);
    }

    public void turnAsync ( double angle){
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle){
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync (Trajectory trajectory){
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory (Trajectory trajectory){
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence){
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence){
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError(){
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update(){
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode){
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior (DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients (DcMotor.RunMode runMode, PIDFCoefficients coefficients){
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower (Pose2d drivePower){
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions () {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v,double v1,double v2,double v3){
        q2.setPower(v);
        q3.setPower(v1);
        q4.setPower(v2);
        q1.setPower(v3);
    }

    @Override
    public double getRawExternalHeading () {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity () {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint ( double maxVel,
                                                                       double maxAngularVel, double trackWidth){
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }
    
    public static TrajectoryAccelerationConstraint getAccelerationConstraint ( double maxAccel){
        return new ProfileAccelerationConstraint(maxAccel);
    }


    public void resetDriveEncoders(){
        q2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        q3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        q1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        q4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }




  
}

