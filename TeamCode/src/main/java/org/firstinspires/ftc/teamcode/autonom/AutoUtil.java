package org.firstinspires.ftc.teamcode.autonom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
    public static void liftPosition(DcMotorEx liftMotor, int pos) {
        liftMotor.setTargetPosition(pos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1f);
    }
    public int getPlatePos(DcMotorEx motor)  {
        int pos = motor.getCurrentPosition();
        return pos;
    }

}
