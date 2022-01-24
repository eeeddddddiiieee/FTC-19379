package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop;

public class depositStateMachine extends LinearOpMode {
    public depositState dstate1;
    public lift robotlift;
    public boolean intakeMode;
    public enum depositState{
        START,
        PRIME,
        MID,
        HIGH,
        DUMP,
        RETRACT
    }
    public void initDeposit(){
        dstate1= depositState.START;
        robotlift=new lift(hardwareMap);
        robotlift.init(hardwareMap);
        intakeMode=true;
    }
    public void runOpMode(){

    }
    public void deposit(hwMecanum robot){
        switch (dstate1) {
            case START:
                robot.bucket.setPosition(hwMecanum.bucketDown);
                robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                if (gamepad1.a){
                    dstate1= depositState.PRIME;
                }
                break;
            case PRIME:
                robot.depositServo.setPosition(hwMecanum.depositClosed);
                robot.bucket.setPosition(hwMecanum.bucketRaised);
                robot.intakeServo.setPosition(hwMecanum.intakeUp);
                intakeMode=false;
                robot.intake.setPower(-1);
                if (gamepad1.a){
                    dstate1= depositState.HIGH;
                }
                if (gamepad1.y){
                    dstate1= depositState.MID;
                }
                if (gamepad1.x){
                    dstate1= depositState.DUMP;
                }
                break;
            case MID:
                intakeMode=true;
                robotlift.setPosition(lift.liftHeight.Med);
                if (robotlift.getHeight()>384.5*8/(1.9685*3.14)) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case HIGH:
                intakeMode=true;
                robotlift.setPosition(lift.liftHeight.High);
                if (robotlift.getHeight()>384.5*12/(1.9685*3.14)) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case DUMP:
                robot.bucket.setPosition(hwMecanum.bucketOut);
                if (gamepad1.x){
                    robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                    sleep(400);
                    dstate1= depositState.RETRACT;
                }
                break;
            case RETRACT:
                robot.bucket.setPosition(hwMecanum.bucketRaised);
                robotlift.resetLift();
                if (robotlift.getHeight()<50)
                {
                    robot.bucket.setPosition(hwMecanum.bucketDown);
                }
            default: dstate1= depositState.START;
        }
        if (gamepad1.right_stick_button && dstate1 != depositState.START) {
            dstate1 = depositState.START;
        }

    }
}
