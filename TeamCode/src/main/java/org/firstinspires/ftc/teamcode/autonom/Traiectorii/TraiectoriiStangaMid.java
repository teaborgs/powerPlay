package org.firstinspires.ftc.teamcode.autonom.Traiectorii;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.autonom.AutonomStangaMid;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;


public class TraiectoriiStangaMid
{
	AutonomStangaMid auto;
	TrajectorySequence deliverPreload,catchCone1,deliverCone1,deliverCone2,catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;

	public TraiectoriiStangaMid(AutonomStangaMid auto)
	{
		this.auto = auto;
		InitTrajectories();
	}

	public void InitTrajectories()
	{
		TrajectoryVelocityConstraint velConstraint = SampleMecanumDrive.getVelocityConstraint(MAX_VEL_AUTO, MAX_ANG_VEL, TRACK_WIDTH);
		TrajectoryAccelerationConstraint accelConstraint = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL_AUTO);

		deliverPreload = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.strafeRight(45)
				.splineToConstantHeading(new Vector2d(-9, -55), Math.toRadians(0))
//				.strafeRight(55.4)
//				.back(8)
				.addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
				.addTemporalMarker(0.5, () -> AutoUtil.platePosition(auto.wormMotor,-2020))
				.build();

		catchCone1 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(26.2)
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
				.build();

		deliverCone1 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.back(25)
				.addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
				.addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.wormMotor,-2020))
				.build();

		catchCone2 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(26)
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
				.build();

		deliverCone2 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.back(25.5)
				.addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
				.addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.wormMotor,-2020))
				.build();

		catchCone3 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(25.9)
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
				.build();

		deliverCone3 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.back(25)
				.addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
				.addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.wormMotor,-2020))
				.build();

		catchCone4 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(26.25)
				.addTemporalMarker( .1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
				.build();

		deliverCone4 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.back(25)
				.addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
				.addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.wormMotor,-2020))
				.build();

		catchCone5 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(26)
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZEROsub))
				.build();

		deliverCone5 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.back(26)
				.addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
				.addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.wormMotor,-2020))
				.build();



		park3 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.back(13.5)
				.turn(Math.toRadians(90))
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
				.build();

		park2 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(9)
				.turn(Math.toRadians(90))
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(1.7, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
				.build();

		park1 = auto.mecanumDrive.trajectorySequenceBuilder(new Pose2d()).setConstraints(velConstraint, accelConstraint)
				.forward(32)
				.addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.wormMotor,-1745))
				.turn(Math.toRadians(90))
				.addTemporalMarker(2.4, () -> AutoUtil.platePosition(auto.wormMotor,-1045))
				.addTemporalMarker(2.5, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
				.build();
	}

	public void RunAuto(int detection)
	{
		AutoUtil.setClaw(auto.catcher,false);

		auto.mecanumDrive.followTrajectorySequence(deliverPreload);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(300);
		AutoUtil.setClaw(auto.catcher,true);
		auto.sleep(200);

		auto.mecanumDrive.followTrajectorySequence(catchCone1);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		AutoUtil.setClaw(auto.catcher,false);
		auto.sleep(300);

		auto.mecanumDrive.followTrajectorySequence(deliverCone1);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		AutoUtil.setClaw(auto.catcher,true);
		auto.sleep(150);

		auto.mecanumDrive.followTrajectorySequence(catchCone2);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		AutoUtil.setClaw(auto.catcher,false);
		auto.sleep(300);

		auto.mecanumDrive.followTrajectorySequence(deliverCone2);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		AutoUtil.setClaw(auto.catcher,true);
		auto.sleep(150);

		auto.mecanumDrive.followTrajectorySequence(catchCone3);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		AutoUtil.setClaw(auto.catcher,false);
		auto.sleep(300);

		auto.mecanumDrive.followTrajectorySequence(deliverCone3);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		AutoUtil.setClaw(auto.catcher,true);
		auto.sleep(150);

		auto.mecanumDrive.followTrajectorySequence(catchCone4);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(100);
		AutoUtil.setClaw(auto.catcher,false);
		auto.sleep(300);

		auto.mecanumDrive.followTrajectorySequence(deliverCone4);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		AutoUtil.setClaw(auto.catcher,true);
		auto.sleep(150);

		auto.mecanumDrive.followTrajectorySequence(catchCone5);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(150);
		AutoUtil.setClaw(auto.catcher,false);
		auto.sleep(300);

		auto.mecanumDrive.followTrajectorySequence(deliverCone5);
		auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		auto.sleep(250);
		AutoUtil.setClaw(auto.catcher,true);
		auto.sleep(150);

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