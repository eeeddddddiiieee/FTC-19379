package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class depositStateMachine extends LinearOpMode {
    public depositState dstate1;
    public lift robotlift;
    public boolean intakeMode;
    public static PIDCoefficients PID = new PIDCoefficients(0.008, 0, 0.0005);
    public PIDFController liftController1;
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
    public void updatePID(hwMecanum r1){
        robotlift.updateLift(r1.lift1.getCurrentPosition());
    }
    public void setLiftPosition(double pos){
        liftController1.setTargetPosition(Range.clip(pos, 0, 800));
    }


    public void deposit(hwMecanum robot){
        switch (dstate1) {
            case START:
                robot.bucket.setPosition(hwMecanum.bucketDown);
                robot.intakeServo.setPosition(hwMecanum.intakeDown);
                robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                if (gamepad1.a){
                    dstate1= depositState.PRIME;
                }
                break;
            case PRIME:
                robot.depositServo.setPosition(hwMecanum.depositClosed);
                robot.intakeServo.setPosition(hwMecanum.intakeUp);
                intakeMode=false;
                robot.intake.setPower(-.5);
                robot.bucket.setPosition(hwMecanum.bucketRaised);

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
                //setLiftPosition(350);
                if (robotlift.getHeight()>384.5*8/(1.9685*3.14)) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case HIGH:
                intakeMode=true;
                robotlift.setPosition(lift.liftHeight.High);
                if (robotlift.getHeight()>750) {
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
                robotlift.setHeight(0, lift.resetMode.YES);
                if (robotlift.getHeight()<50)
                {
                    robot.bucket.setPosition(hwMecanum.bucketDown);

                }
                dstate1=depositState.START;
                break;

            default: dstate1= depositState.START;
        }
        
        if (gamepad1.right_stick_button && dstate1 != depositState.START) {
            dstate1 = depositState.START;
        }

    }
    public depositState getDepositState(){
        return dstate1;
    }
}
