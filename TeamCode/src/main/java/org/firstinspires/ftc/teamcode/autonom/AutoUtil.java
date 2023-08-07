package org.firstinspires.ftc.teamcode.autonom;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoUtil {
	SampleMecanumDrive mecanumDrive;
	public static void setClaw(Servo servo, boolean open){
		double pos = open ? 0f : 0.6f;
		servo.setPosition(pos);

	}
	public static void platePosition(DcMotorEx plateMotor, int pos){
		plateMotor.setTargetPosition(pos);
		plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		plateMotor.setPower(1f);
	}
	public static void liftPosition(DcMotorEx liftMotor1, DcMotorEx liftMotor2, AutoPosition pos) {
		if(pos == AutoPosition.HIGH) {
			liftMotor1.setTargetPosition(-1145);
			liftMotor2.setTargetPosition(-1145);
		}
		else if(pos == AutoPosition.HIGHother) {
			liftMotor1.setTargetPosition(-1115);
			liftMotor2.setTargetPosition(-1115); //1100
		}
		else if(pos == AutoPosition.HIGHaf) {
			liftMotor1.setTargetPosition(-1120);
			liftMotor2.setTargetPosition(-1120); //1100
		}
		else if(pos == AutoPosition.ZEROsub) {
			liftMotor1.setTargetPosition(-15);
			liftMotor2.setTargetPosition(-15);
		}
		else if(pos == AutoPosition.MID) {
			liftMotor1.setTargetPosition(-850);
			liftMotor2.setTargetPosition(-850);
		}
		else if(pos == AutoPosition.MIDother) {
			liftMotor1.setTargetPosition(-845);
			liftMotor2.setTargetPosition(-845);
		}
		else if(pos == AutoPosition.LOW) {
			liftMotor1.setTargetPosition(-480);
			liftMotor2.setTargetPosition(-480);
		}
		else if(pos == AutoPosition.ZERO) {
			liftMotor1.setTargetPosition(0);
			liftMotor2.setTargetPosition(0);
		}
		else if(pos == AutoPosition.CONE5) {
			liftMotor1.setTargetPosition(-200);
			liftMotor2.setTargetPosition(-196);
		}
		else if(pos == AutoPosition.CONE4) {
			liftMotor1.setTargetPosition(-164);
			liftMotor2.setTargetPosition(-168);
		}
		else if(pos == AutoPosition.CONE3) {
			liftMotor1.setTargetPosition(-114);
			liftMotor2.setTargetPosition(-125);
		}
		else if(pos == AutoPosition.CONE2) {
			liftMotor1.setTargetPosition(-72);
			liftMotor2.setTargetPosition(-74);
		}

		liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotor1.setPower(0.6f);
		liftMotor2.setPower(0.6f);
	}
	public static void relaxMotor(DcMotorEx liftMotor1, DcMotorEx liftMotor2) {
		liftMotor1.setPower(0f);
		liftMotor2.setPower(0f);
	}

}
