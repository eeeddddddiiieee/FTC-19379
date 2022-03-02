package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class depositStateMachine {
    public depositState dstate1;
    public lift robotlift;
    public boolean intakeMode;
    public double intakePower;
    public static PIDCoefficients PID = new PIDCoefficients(0.008, 0, 0.0005);
    public PIDFController liftController1;
    public enum depositState{
        START,
        PRIME,
        MID,
        HIGH,
        DUMP,
        EXTENDED_DUMP,
        RETRACT
    }
    public void initDeposit(HardwareMap hardwareMap){
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

    public void setState(depositState d1){
        dstate1=d1;
    }
    public void deposit(hwMecanum robot, Gamepad gamepad1)throws InterruptedException{
        switch (dstate1) {
            case START:
                robot.intakeMode=true;
                robot.bucket.setPosition(hwMecanum.bucketDown);
                robot.depositExtension.setPosition(robot.depositRetracted);
                robot.intakeServo.setPosition(hwMecanum.intakeDown);
                robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                if (robot.isCargo==true||gamepad1.start){
                    dstate1= depositState.PRIME;
                }
                break;
            case PRIME:
                robot.intakeMode=false;
                intakePower=.5;
                robot.depositServo.setPosition(hwMecanum.depositClosed);
                robot.intakeServo.setPosition(hwMecanum.intakeUp);
                robot.red1.setState(true);
                robot.bucket.setPosition(hwMecanum.bucketRaised);

                if (gamepad1.right_stick_button){
                    dstate1= depositState.HIGH;
                }
                if (gamepad1.x){
                    dstate1= depositState.MID;
                }
                if (gamepad1.b){
                    dstate1= depositState.DUMP;
                }
                if (gamepad1.back){
                    dstate1=depositState.EXTENDED_DUMP;
                }
                break;
            case MID:
                robot.intakeMode=true;
                robotlift.setHeight(320,lift.resetMode.NO);
                robot.depositExtension.setPosition(robot.depositExtended);

                robot.red1.setState(false);
                robot.green1.setState(true);
                //setLiftPosition(350);
                if (robotlift.getHeight()>250) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case HIGH:
                robot.intakeMode=true;
                robot.red1.setState(false);
                robot.green1.setState(true);
                robotlift.setHeight(1100,lift.resetMode.NO);
                robot.depositExtension.setPosition(robot.depositExtended);

                if (robotlift.getHeight()>750) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case EXTENDED_DUMP:
                intakePower=0;
                robot.bucket.setPosition(hwMecanum.bucketOut);
                robot.depositExtension.setPosition(robot.depositExtended);
                if (gamepad1.right_stick_button){
                    robot.depositServo.setPosition(hwMecanum.depositOpen+.2);
                    sleep(400);
                    robot.bucket.setPosition(hwMecanum.bucketRaised);
                    if (robot.bucket.getPosition()>.4){
                        dstate1= depositState.RETRACT;
                    }
                }
                break;
            case DUMP:
                intakePower=0;
                robot.bucket.setPosition(hwMecanum.bucketOut);
                if (gamepad1.back){
                    dstate1=depositState.EXTENDED_DUMP;
                }
                if (gamepad1.right_stick_button){
                    robot.depositServo.setPosition(hwMecanum.depositOpen+.2);
                    sleep(400);
                    robot.bucket.setPosition(hwMecanum.bucketRaised);
                    if (robot.bucket.getPosition()>.4){
                    dstate1= depositState.RETRACT;
                    }
                }
                break;
            case RETRACT:
                robot.intakeMode=true;
                robot.bucket.setPosition(hwMecanum.bucketRaised);
                robot.green1.setState(false);
                robot.depositExtension.setPosition(robot.depositRetracted);
                robotlift.setHeight(0, lift.resetMode.YES);

                if (robotlift.getHeight()<50&&robot.depositExtension.getPosition()==robot.depositRetracted)
                {
                    robot.bucket.setPosition(hwMecanum.bucketDown);

                }
                dstate1=depositState.START;
                break;

            default: dstate1= depositState.START;
        }
        
        if (gamepad1.y && dstate1 != depositState.RETRACT) {
            dstate1 = depositState.RETRACT;
        }

    }

    public void deposit(hwMecanum robot, int signal)throws InterruptedException{
        switch (dstate1) {
            case START:
                intakeMode=true;
                robot.bucket.setPosition(hwMecanum.bucketDown);
                robot.intakeServo.setPosition(hwMecanum.intakeDown);
                robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                robot.depositExtension.setPosition(robot.depositRetracted);

                if (robot.isCargo==true){
                    dstate1= depositState.PRIME;
                }
                break;
            case PRIME:
                robot.depositServo.setPosition(hwMecanum.depositClosed);
                robot.intakeServo.setPosition(hwMecanum.intakeUp);
                robot.bucket.setPosition(hwMecanum.bucketRaised);
                if (robot.bucket.getPosition()<.65){
                    robot.intakeMode=false;
                    intakePower=-.5;
                }
                if (signal==2){
                    dstate1= depositState.HIGH;
                }
                if (signal==3){
                    dstate1= depositState.MID;
                }
                if (signal==4){
                    dstate1= depositState.DUMP;
                }
                if (signal==6){
                    dstate1=depositState.EXTENDED_DUMP;
                }
                break;
            case MID:
                robot.intakeMode=false;
                robotlift.setHeight(370,lift.resetMode.NO);
                robot.depositExtension.setPosition(robot.depositExtended);

                //setLiftPosition(350);
                if (robotlift.getHeight()>250) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case HIGH:
                robot.intakeMode=false;
                robotlift.setHeight(1050,lift.resetMode.NO);
                robot.depositExtension.setPosition(robot.depositExtended);

                if (robotlift.getHeight()>750) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case DUMP:
                intakePower=-.5;
                robot.bucket.setPosition(hwMecanum.bucketOut);
                if (signal==5){
                    robot.depositServo.setPosition(hwMecanum.depositOpen+.2);
                    sleep(600);
                    robot.bucket.setPosition(hwMecanum.bucketRaised);
                    if (robot.bucket.getPosition()>.45){
                        dstate1= depositState.RETRACT;
                    }
                }
                break;
            case EXTENDED_DUMP:

                intakePower=-.5;
                robot.bucket.setPosition(hwMecanum.bucketOut);
                robot.depositExtension.setPosition(robot.depositExtended);
                if (signal==5){
                    robot.depositServo.setPosition(hwMecanum.depositOpen+.2);
                    sleep(400);
                    robot.bucket.setPosition(hwMecanum.bucketRaised);
                    if (robot.bucket.getPosition()>.4){
                        dstate1= depositState.RETRACT;
                    }
                }
                break;
            case RETRACT:
                robot.intakeMode=true;
                robot.bucket.setPosition(hwMecanum.bucketRaised);

                robot.depositExtension.setPosition(robot.depositRetracted);
                robotlift.setHeight(0, lift.resetMode.YES);

                if (robotlift.getHeight()<50&&robot.depositExtension.getPosition()==robot.depositRetracted)
                {
                    robot.bucket.setPosition(hwMecanum.bucketDown);

                }
                dstate1=depositState.START;
                break;

            default: dstate1= depositState.START;
        }
    }

    public depositState getDepositState(){
        return dstate1;
    }
}