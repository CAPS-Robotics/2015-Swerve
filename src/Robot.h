#ifndef __MY_ROBOT_H_
#define __MY_ROBOT_H_

#include <pthread.h>

void* driveFunc(void* arg);
void* inputFunc(void* arg);
void* swerveSyncFunc(void* arg);

RobotDrive      *drive;
Joystick        *joystick;

Encoder			*swerve_enc_fl;
Encoder			*swerve_enc_fr;
Encoder			*swerve_enc_bl;
Encoder			*swerve_enc_br;

PWM				*swerve_rot_fl;
PWM				*swerve_rot_fr;
PWM				*swerve_rot_bl;
PWM				*swerve_rot_br;

PWM				*swerve_mov_fl;
PWM				*swerve_mov_fr;
PWM				*swerve_mov_bl;
PWM				*swerve_mov_br;

bool             driveRun;
pthread_t        driveThread;
pthread_t		 inputThread;
pthread_t		 macroThread;
pthread_t        swerveSyncThread;

int sgn(double num) {
	return num == 0 ? 0 : num / fabs(num);
}

int safe_enc(int enc_val) {
	if (enc_val < 0) {
		enc_val %= 415;
		enc_val += 415;
	}
	return enc_val % 415;
}

short safe_pwm(short pwm_val) {
	if (pwm_val > 1999) {
		pwm_val = 1999;
	}
	else if (pwm_val < 1) {
		pwm_val = 1;
	}
	return pwm_val;
}

int deg_to_enc(double a) {
	return (int) ((a / 360.f) * 415.f);
}

bool enc_dir(double a1, double a2) {
	//T = CW
	//F = CCW
	if (a1 < deg_to_enc(90) && a2 > deg_to_enc(270)) {
		return false;
	}
	else if (a1 > deg_to_enc(270) && a2 < deg_to_enc(90)) {
		return true;
	}
	return a2 > a1;
}

void rot_from_enc(Encoder* enc, PWM* pwm, int ref) {
	if (enc_dir(safe_enc(enc->Get()), ref)) {
		pwm->SetRaw(safe_pwm(1000 - (fabs(safe_enc(enc->Get()) - ref) * 15)));
	}
	else if (!enc_dir(safe_enc(enc->Get()), ref)) {
		pwm->SetRaw(safe_pwm(1000 + (fabs(ref - safe_enc(enc->Get())) * 15)));
	}
}

float vect_mag(double x, double y) {
	return sqrt(pow(x, 2) + pow(y, 2));
}

class SwerveTest : public IterativeRobot
{
public:
    SwerveTest() {};
    ~SwerveTest() {};

    void RobotInit();
    void AutonomousInit();
    void TeleopInit();
    void DisabledInit();

    // unused functions
    void TestInit() {}
    void DisabledPeriodic() {}
    void AutonomousPeriodic() {}
    void TeleopPeriodic() {}
    void TestPeriodic() {}
};

#endif
