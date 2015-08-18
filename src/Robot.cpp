#include <unistd.h>

#include "WPILib.h"
#include "Robot.h"
#include "config.h"
#include "time.h"

void SwerveTest::RobotInit() {
	joystick = new Joystick(JOY_PORT_0);
	swerve_enc_fl = new Encoder(SWERVE_FL_0, SWERVE_FL_1);
	swerve_enc_fr = new Encoder(SWERVE_FR_0, SWERVE_FR_1);
	swerve_enc_bl = new Encoder(SWERVE_BL_0, SWERVE_BL_1);
	swerve_enc_br = new Encoder(SWERVE_BR_0, SWERVE_BR_1);
	swerve_rot_fl = new PWM(SWERVE_ROT_FL);
	swerve_rot_fr = new PWM(SWERVE_ROT_FR);
	swerve_rot_bl = new PWM(SWERVE_ROT_BL);
	swerve_rot_br = new PWM(SWERVE_ROT_BR);
	swerve_mov_fl = new PWM(SWERVE_MOV_FL);
	swerve_mov_fr = new PWM(SWERVE_MOV_FR);
	swerve_mov_bl = new PWM(SWERVE_MOV_BL);
	swerve_mov_br = new PWM(SWERVE_MOV_BR);
	pthread_create(&driveThread, NULL, driveFunc, NULL);
	pthread_create(&inputThread, NULL, inputFunc, NULL);
	pthread_create(&swerveSyncThread, NULL, swerveSyncFunc, NULL);
}

void SwerveTest::TeleopInit() {
	driveRun = true;
}

void SwerveTest::AutonomousInit() {

}

void SwerveTest::DisabledInit() {
	driveRun = false;
}

void* driveFunc(void* args) {
	/*float Kp = 0.024000; //A
	float Ki = 0.021000; //O
	float Kd = 0;        //L
	double oldTime = GetTime();
	float movPIDError = 0;
	float movPIDIntegral = 0;
	float movCurrentSpeed = 0;*/
	while(true) {
		SmartDashboard::PutString("DB/String 0", std::to_string(safe_enc(swerve_enc_fl->Get())));
		SmartDashboard::PutString("DB/String 1", std::to_string(safe_enc(swerve_enc_bl->Get())));
		SmartDashboard::PutString("DB/String 5", std::to_string(safe_enc(swerve_enc_fr->Get())));
		SmartDashboard::PutString("DB/String 6", std::to_string(safe_enc(swerve_enc_br->Get())));
		if (driveRun) {
			/*double ctime = GetTime();
			float movCurrentError = joystick->GetRawAxis(JOY_AXIS_RY) - movCurrentSpeed;
			movPIDIntegral += movPIDError * (ctime - oldTime);
			float movPIDDerivative = (movCurrentError - movPIDError) / (ctime - oldTime);
			movCurrentSpeed += (Kp * movCurrentError) + (Ki * movPIDIntegral) + (Kd * movPIDDerivative);
			movPIDError = movCurrentError;*/
			//Calibration code
			/*swerve_rot_fl->SetRaw(1000 - (int) (joystick->GetRawAxis(JOY_AXIS_LX) * 999));
			swerve_rot_fr->SetRaw(1000 - (int) (joystick->GetRawAxis(JOY_AXIS_LX) * 999));
			swerve_rot_bl->SetRaw(1000 - (int) (joystick->GetRawAxis(JOY_AXIS_LX) * 999));
			swerve_rot_br->SetRaw(1000 - (int) (joystick->GetRawAxis(JOY_AXIS_LX) * 999));*/
			//swerve_rot_fl->SetRaw(1000 - (int) (joystick->GetRawAxis(JOY_AXIS_LX) * 999));
			/*int target = 1000 - (joystick->GetRawAxis(JOY_AXIS_RY) * 100);
			swerve_mov_fl->SetRaw(target);
			swerve_mov_fr->SetRaw(target);
			swerve_mov_bl->SetRaw(target);
			swerve_mov_br->SetRaw(target);*/
		}
	}
}

void* inputFunc(void* args) {
	while(true) {

	}
}

void* swerveSyncFunc(void* args) {
	float deadZone = 0.05;
	while(true) {
		int enc_ref_l = safe_enc(swerve_enc_fl->Get());
		int enc_ref_r = enc_ref_l; //temp until control is ready
		if (driveRun) {
			if (fabs(joystick->GetRawAxis(JOY_AXIS_LX)) > deadZone || fabs(joystick->GetRawAxis(JOY_AXIS_LY)) > deadZone) {
				//use arithmetic on enc_ref in case both sticks are in use
				double rad = atan2(joystick->GetRawAxis(JOY_AXIS_LX), -joystick->GetRawAxis(JOY_AXIS_LY));
				double deg = rad * (180 / M_PI);
				if (deg < 0) {
					deg += 360;
				}
				enc_ref_l = deg_to_enc(deg);
				enc_ref_r = deg_to_enc(deg);
			}
			SmartDashboard::PutString("DB/String 3", std::to_string(joystick->GetRawAxis(JOY_AXIS_RX)));
			if (fabs(joystick->GetRawAxis(JOY_AXIS_RX)) > deadZone) {
				int target_l = 1000 - (joystick->GetRawAxis(JOY_AXIS_RX) * 200);
				int target_r = 1000 + (joystick->GetRawAxis(JOY_AXIS_RX) * 200);
				swerve_mov_fl->SetRaw(target_l);
				swerve_mov_fr->SetRaw(target_r);
				swerve_mov_bl->SetRaw(target_l);
				swerve_mov_br->SetRaw(target_r);
			}
		}
		SmartDashboard::PutString("DB/String 2", std::to_string(enc_ref_l));
		SmartDashboard::PutString("DB/String 7", std::to_string(enc_ref_r));
		rot_from_enc(swerve_enc_fl, swerve_rot_fl, enc_ref_l);
		rot_from_enc(swerve_enc_fr, swerve_rot_fr, enc_ref_r);
		rot_from_enc(swerve_enc_bl, swerve_rot_bl, enc_ref_l);
		rot_from_enc(swerve_enc_br, swerve_rot_br, enc_ref_r);
		float mag = vect_mag(joystick->GetRawAxis(JOY_AXIS_LX), joystick->GetRawAxis(JOY_AXIS_LY));
		int target = 1000 - (mag * 100);
		swerve_mov_fl->SetRaw(target);
		swerve_mov_fr->SetRaw(target);
		swerve_mov_bl->SetRaw(target);
		swerve_mov_br->SetRaw(target);
	}
}

START_ROBOT_CLASS(SwerveTest);
