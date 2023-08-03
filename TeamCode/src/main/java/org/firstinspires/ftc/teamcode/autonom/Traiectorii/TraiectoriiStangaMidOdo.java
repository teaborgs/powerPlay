package org.firstinspires.ftc.teamcode.autonom.Traiectorii;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoStMidOdo;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.autonom.AutonomStangaMid;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class TraiectoriiStangaMidOdo
{
	AutoStMidOdo auto;
	TrajectorySequence deliverPreload,catchCone1,deliverCone1, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;

	public TraiectoriiStangaMidOdo(AutoStMidOdo auto)
	{
		this.auto = auto;
		initializeTrajectories();
	}

	public void initializeTrajectories()
	{
		deliverPreload = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.strafeRight(45)
				.splineToConstantHeading(new Vector2d(-9, -55), Math.toRadians(0))
				//.strafeRight(55.4)
				//.back(7.5)
				.build();
		catchCone1 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(26.2)
				.build();
		deliverCone1 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.back(25)
				.build();
		catchCone2 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(26)
				.build();
		deliverCone2 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.back(25.5)
				.build();
		catchCone3 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(25.9)
				.build();
		deliverCone3 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.back(25)
				.build();
		catchCone4 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(26.25)
				.build();
		deliverCone4 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.back(25)
				.build();
		catchCone5 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(26)
				.build();
		deliverCone5 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.back(26) /// neagra
				.build();
		park3 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.back(13.5)
				.turn(Math.toRadians(90))
				.build();
		park2 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(11)
				.turn(Math.toRadians(90))
				.build();
		park1 = auto.mecanumDrive
				.trajectorySequenceBuilder(new Pose2d())
				.setConstraints
						(SampleMecanumDrive.getVelocityConstraint(35,MAX_ANG_VEL,TRACK_WIDTH)
								,SampleMecanumDrive.getAccelerationConstraint(35))
				.forward(32)
				.turn(Math.toRadians(90))
				.build();
	}

	public void runAuto(int detected) {
		auto.mecanumDrive.followTrajectorySequence(deliverPreload);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(300);
		auto.sleep(100);
		auto.mecanumDrive.followTrajectorySequence(catchCone1);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		auto.sleep(300);
		auto.mecanumDrive.followTrajectorySequence(deliverCone1);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		auto.sleep(100);
		auto.mecanumDrive.followTrajectorySequence(catchCone2);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		auto.sleep(300);
		auto.mecanumDrive.followTrajectorySequence(deliverCone2);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		auto.sleep(100);
		auto.mecanumDrive.followTrajectorySequence(catchCone3);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		auto.sleep(300);
		auto.mecanumDrive.followTrajectorySequence(deliverCone3);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		auto.sleep(100);
		auto.mecanumDrive.followTrajectorySequence(catchCone4);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		auto.sleep(300);
		auto.mecanumDrive.followTrajectorySequence(deliverCone4);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		auto.sleep(100);
		auto.mecanumDrive.followTrajectorySequence(catchCone5);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(150);
		auto.sleep(300);
		auto.mecanumDrive.followTrajectorySequence(deliverCone5);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		auto.sleep(100);

		if(detected == 1)
			auto.mecanumDrive.followTrajectorySequence(park1);
		else if(detected == 2)
			auto.mecanumDrive.followTrajectorySequence(park2);
		else
			auto.mecanumDrive.followTrajectorySequence(park3);
	}
}