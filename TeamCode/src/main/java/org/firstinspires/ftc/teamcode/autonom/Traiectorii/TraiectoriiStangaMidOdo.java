package org.firstinspires.ftc.teamcode.autonom.Traiectorii;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL_AUTO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL_AUTO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.autonom.AutoStMidOdo;
import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class TraiectoriiStangaMidOdo
{
	AutoStMidOdo auto;
	TrajectorySequence deliverPreload,catchCone1,deliverCone1,deliverCone2,catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;

	public TraiectoriiStangaMidOdo(AutoStMidOdo auto)
	{
		this.auto = auto;
		InitTrajectories();
	}

	public void InitTrajectories()
	{
		TrajectoryVelocityConstraint velocityConstraint = OdometryMecanumDrive.getVelocityConstraint(MAX_VEL_AUTO, MAX_ANG_VEL, TRACK_WIDTH);
		TrajectoryAccelerationConstraint accelerationConstraint = OdometryMecanumDrive.getAccelerationConstraint(MAX_ACCEL_AUTO);

		deliverPreload = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velocityConstraint, accelerationConstraint)
				.strafeRight(45)
				.splineToConstantHeading(new Vector2d(-9, -55), Math.toRadians(0))
				//.strafeRight(55.4)
				//.back(7.5)
				.build();

		catchCone1 = auto.mecanumDrive.trajectorySequenceBuilder(deliverPreload.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(26.2)
				.build();

		deliverCone1 = auto.mecanumDrive.trajectorySequenceBuilder(catchCone1.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.back(25)
				.build();

		catchCone2 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone1.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(26)
				.build();

		deliverCone2 = auto.mecanumDrive.trajectorySequenceBuilder(catchCone2.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.back(25.5)
				.build();

		catchCone3 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone2.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(25.9)
				.build();

		deliverCone3 = auto.mecanumDrive.trajectorySequenceBuilder(catchCone3.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.back(25)
				.build();

		catchCone4 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone3.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(26.25)
				.build();

		deliverCone4 = auto.mecanumDrive.trajectorySequenceBuilder(catchCone4.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.back(25)
				.build();

		catchCone5 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone4.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(26)
				.build();

		deliverCone5 = auto.mecanumDrive.trajectorySequenceBuilder(catchCone5.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.back(26)
				.build();



		park3 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone5.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.back(13.5)
				.turn(Math.toRadians(90))
				.build();

		park2 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone5.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(11)
				.turn(Math.toRadians(90))
				.build();

		park1 = auto.mecanumDrive.trajectorySequenceBuilder(deliverCone5.end()).setConstraints(velocityConstraint, accelerationConstraint)
				.forward(32)
				.turn(Math.toRadians(90))
				.build();
	}

	public void RunAuto(int detection)
	{
		auto.mecanumDrive.followTrajectorySequence(deliverPreload);
		auto.sleep(300);
		auto.sleep(100);
		
		auto.mecanumDrive.followTrajectorySequence(catchCone1);
		auto.sleep(100);
		auto.sleep(300);
		
		auto.mecanumDrive.followTrajectorySequence(deliverCone1);
		auto.sleep(250);
		auto.sleep(100);
		
		auto.mecanumDrive.followTrajectorySequence(catchCone2);
		auto.sleep(100);
		auto.sleep(300);
		
		auto.mecanumDrive.followTrajectorySequence(deliverCone2);
		auto.sleep(250);
		auto.sleep(100);
		
		auto.mecanumDrive.followTrajectorySequence(catchCone3);
		auto.sleep(100);
		auto.sleep(300);
		
		auto.mecanumDrive.followTrajectorySequence(deliverCone3);
		auto.sleep(250);
		auto.sleep(100);
		
		auto.mecanumDrive.followTrajectorySequence(catchCone4);
		auto.sleep(100);
		auto.sleep(300);
		
		auto.mecanumDrive.followTrajectorySequence(deliverCone4);
		auto.sleep(250);
		auto.sleep(100);
		
		auto.mecanumDrive.followTrajectorySequence(catchCone5);
		auto.sleep(150);
		auto.sleep(300);
		
		auto.mecanumDrive.followTrajectorySequence(deliverCone5);
		auto.sleep(250);
		auto.sleep(100);

		// Park
		switch (detection)
		{
			case 1:
				auto.mecanumDrive.followTrajectorySequence(park1);
				break;
			case 2:
				auto.mecanumDrive.followTrajectorySequence(park2);
				break;
			case 3:
				auto.mecanumDrive.followTrajectorySequence(park3);
				break;
		}
	}
}