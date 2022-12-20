package org.firstinspires.ftc.teamcode.autonom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoUtil {
    SampleMecanumDrive mecanumDrive;
    public static void setClaw(Servo servo, boolean open){
        double pos = open?0f:0.4f;
        servo.setPosition(pos);

    }
    public static void platePosition(DcMotorEx plateMotor, int pos){
        plateMotor.setTargetPosition(pos);
        plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        plateMotor.setPower(1f);
    }
    public static void liftPosition(DcMotorEx liftMotor1, DcMotorEx liftMotor2, AutoPosition pos) {
        if(pos == AutoPosition.HIGH) {
            liftMotor1.setTargetPosition(-1757);
            liftMotor2.setTargetPosition(-1764);
        }
        else if(pos == AutoPosition.MID) {
            liftMotor1.setTargetPosition(-1272);
            liftMotor2.setTargetPosition(-1282);
        }
        else if(pos == AutoPosition.LOW) {
            liftMotor1.setTargetPosition(-770);
            liftMotor2.setTargetPosition(-770);
        }
        else if(pos == AutoPosition.ZERO) {
            liftMotor1.setTargetPosition(0);
            liftMotor2.setTargetPosition(0);
        }
        else if(pos == AutoPosition.CONE5) {
            liftMotor1.setTargetPosition(-244);
            liftMotor2.setTargetPosition(-240);
        }
        else if(pos == AutoPosition.CONE4) {
            liftMotor1.setTargetPosition(-219);
            liftMotor2.setTargetPosition(-221);
        }
        else if(pos == AutoPosition.CONE3) {
            liftMotor1.setTargetPosition(-130);
            liftMotor2.setTargetPosition(-131);
        }
        else if(pos == AutoPosition.CONE2) {
            liftMotor1.setTargetPosition(-50);
            liftMotor2.setTargetPosition(-50);
        }

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor1.setPower(1f);
        liftMotor2.setPower(1f);
    }
    public int getPlatePos(DcMotorEx motor)  {
        int pos = motor.getCurrentPosition();
        return pos;
    }

}
