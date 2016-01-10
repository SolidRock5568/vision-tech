#include "WPILib.h"

class Robot: public IterativeRobot {
	Victor open_close_motor; //3
	Jaguar up_down_motor; //2
	Talon right_motor; //1
	Talon left_motor;
	Joystick drive_stick;//0
	Joystick up_down_stick;
	bool yellow;
	bool green;

public:

		//A structure to hold measurements of a particle
	struct ParticleReport {
		double PercentAreaToImageArea;
		double Area;
		double ConvexHullArea;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
	};
	
	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double Trapezoid;
		double LongAspect;
		double ShortAspect;
		double AreaToConvexHullArea;
	};
	
	//Send image to dashboard if IMAQ has not thrown an error
	void SendToDashboard(Image *image, int error) {
		if(error < ERR_SUCCESS) {
			DriverStation::ReportError("Send To Dashboard error: " + std::to_string((long)imaqError) + "\n");
		} else {
			CameraServer::GetInstance()->SetImage(image);
		}
	}
	
	//Comparator function for sorting particles. Returns true if particle 1 is larger
	static bool CompareParticleSizes(ParticleReport particle1, ParticleReport particle2){
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}
	
	/**
	* Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	* linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	*/
	double ratioToScore(double ratio) {
	return (fmax(0, fmin(100*(1-fabs(1-ratio)), 100)));
	}
	
	/*
	* Method to score convex hull area. This scores how "complete" the particle is. Particles with large holes will score worse than a filled in shape
	*/
	double ConvexHullAreaScore(ParticleReport report) {
		return ratioToScore((report.Area/report.ConvexHullArea)*1.18);
	}
	
	/*
	* Method to score if the particle appears to be a trapezoid. Compares the convex hull (filled in) area to the area of the bounding box.
	* The expectation is that the convex hull area is about 95.4% of the bounding box area for an ideal tote.
	*/
	double TrapezoidScore(ParticleReport report) {
		return ratioToScore(report.ConvexHullArea/((report.BoundingRectRight-report.BoundingRectLeft)*(report.BoundingRectBottom-report.BoundingRectTop)*.954));
	}
	
	/**
	* Method to score if the aspect ratio of the particle appears to match the long side of a tote.
	*/
	double LongSideScore(ParticleReport report) {
	return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/LONG_RATIO);
	}
	
	/**
	* Method to score if the aspect ratio of the particle appears to match the short side of a tote.
	*/
	double ShortSideScore(ParticleReport report){
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/SHORT_RATIO);
	}
	
	/**
	* Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
	* showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	*
	* @param image The image to use for measuring the particle estimated rectangle
	* @param report The Particle Analysis Report for the particle
	* @param isLong Boolean indicating if the target is believed to be the long side of a tote
	* @return The estimated distance to the target in feet.
	*/
	double computeDistance (Image *image, ParticleReport report, bool isLong) {
		double normalizedWidth, targetWidth;
		int xRes, yRes;
	
		imaqGetImageSize(image, &xRes, &yRes);
		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/xRes;
		SmartDashboard::PutNumber("Width", normalizedWidth);
		targetWidth = isLong ? 26.9 : 16.9;
	
		return  targetWidth/(normalizedWidth*12*tan(VIEW_ANGLE*M_PI/(180*2)));
	}
	
	IMAQdxSession session;
	Image *frame;
	IMAQdxError Cammera;
	
	//Image *frame;
	Image *binaryFrame;
	int imaqError;
	
	//Constants
	Range TOTE_HUE_RANGE_GREEN = {80, 120};
	Range TOTE_HUE_RANGE_YELLOW = {40, 60};//default hue range for yellow tote
	Range TOTE_SAT_RANGE_YELLOW= {150,255};
	Range TOTE_SAT_RANGE_GREEN = {70,120};//default saturation range for yellow tote
	Range TOTE_VAL_RANGE_YELLOW = {70, 255};
	Range TOTE_VAL_RANGE_GREEN = {20, 100};//Default value range for yellow tote
	double AREA_MINIMUM = 2; //Default Area minimum for particle as a percentage of total image area
	double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
	double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
	double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
	double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	ParticleFilterCriteria2 criteria[1];
	ParticleFilterOptions2 filterOptions = {0,0,1,1};
	Scores scores;
	
Robot() :
	//Tells what ports sensors, motors, or joystick are plugged into
	open_close_motor(3),
	up_down_motor(2),
	right_motor(1),
	left_motor(0),
	drive_stick(0),
	up_down_stick(1)
	
	{ CameraServer ::GetInstance() ->SetQuality(50); }
	
	void RobotInit() {
		binaryFrame = imaqCreateImage(IMAQ_IMAGE_U8, 0);
		//Put default values to SmartDashboard so fields will appear
		//SmartDashboard::PutNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
		//SmartDashboard::PutNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
		//SmartDashboard::PutNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
		//SmartDashboard::PutNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
		//SmartDashboard::PutNumber("Tote val min", TOTE_VAL_RANGE.minValue);
		//SmartDashboard::PutNumber("Tote val max", TOTE_VAL_RANGE.maxValue);
		//SmartDashboard::PutNumber("Area min %", AREA_MINIMUM);
	    // create an image
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		Cammera = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
		if(Cammera != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		Cammera = IMAQdxConfigureGrab(session);
		if(Cammera != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}
	}

	void AutonomousInit() {
		imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &TOTE_HUE_RANGE_YELLOW, &TOTE_SAT_RANGE_YELLOW, &TOTE_VAL_RANGE_YELLOW);
	}

	void AutonomousPeriodic() {
		//Make the camera search for yellow
		imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &TOTE_HUE_RANGE_YELLOW, &TOTE_SAT_RANGE_YELLOW, &TOTE_VAL_RANGE_YELLOW);

		//Send particle count to dashboard
		int numParticles = 0;
		imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
		SmartDashboard::PutNumber("Masked particles", numParticles);

		//Send masked image to dashboard to assist in tweaking mask.
		SendToDashboard(binaryFrame, imaqError);

		float areaMin = SmartDashboard::GetNumber("Area min %", AREA_MINIMUM);
		criteria[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, areaMin, 100, false, false};
		imaqError = imaqParticleFilter4(binaryFrame, binaryFrame, criteria, 1, &filterOptions,NULL,NULL);

		//Send particle count after filtering to dashboard
		imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
		SmartDashboard::PutNumber("Filtered particles", numParticles);

		//If the number of yellow particles > than 20, move the right motor forward for 4000 milliseconds.
		if(numParticles > 20){
			right_motor.Set(.2);
			Wait(4000);
			right_motor.Set(0);
		}
		else{
			right_motor.Set(0);
		}
	}

	void TeleopInit() {
		//basevalue = 0;
		//maxvalue = 0;
		green = false;
		yellow = false;
	}

	void TeleopPeriodic() {
		IMAQdxStartAcquisition(session);
        // grab an image, draw the circle, and provide it for the camera server which will
        // in turn send it to the dashboard.
		while(IsOperatorControl() && IsEnabled()) {
			IMAQdxGrab(session, frame, true, NULL);
			if(Cammera != IMAQdxErrorSuccess) {
				DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
			} else {
				//imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
				CameraServer::GetInstance()->SetImage(frame);
				//TOTE_HUE_RANGE.minValue = SmartDashboard::GetNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
				//TOTE_HUE_RANGE.maxValue = SmartDashboard::GetNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
				//TOTE_SAT_RANGE.minValue = SmartDashboard::GetNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
				//TOTE_SAT_RANGE.maxValue = SmartDashboard::GetNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
				//TOTE_VAL_RANGE.minValue = SmartDashboard::GetNumber("Tote val min", TOTE_VAL_RANGE.minValue);
				//TOTE_VAL_RANGE.maxValue = SmartDashboard::GetNumber("Tote val max", TOTE_VAL_RANGE.maxValue);

				//Threshold the image looking for yellow (tote color)
				if (yellow == true)
					{imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &TOTE_HUE_RANGE_YELLOW, &TOTE_SAT_RANGE_YELLOW, &TOTE_VAL_RANGE_YELLOW);
				}

				if (green == true)
					{imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &TOTE_HUE_RANGE_GREEN, &TOTE_SAT_RANGE_GREEN, &TOTE_VAL_RANGE_GREEN);
				}
				//Send particle count to dashboard
				int numParticles = 0;
				imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
				SmartDashboard::PutNumber("Masked particles", numParticles);

				//Send masked image to dashboard to assist in tweaking mask.
				SendToDashboard(binaryFrame, imaqError);

				float areaMin = SmartDashboard::GetNumber("Area min %", AREA_MINIMUM);
				criteria[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, areaMin, 100, false, false};
				imaqError = imaqParticleFilter4(binaryFrame, binaryFrame, criteria, 1, &filterOptions,NULL,NULL);

				//Send particle count after filtering to dashboard
				imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
				SmartDashboard::PutNumber("Filtered particles", numParticles);

				if(numParticles > 0) {
					//Measure particles and sort by particle size
					std::vector<ParticleReport> particles;
					for(int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
						ParticleReport par;
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &(par.PercentAreaToImageArea));
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA, &(par.Area));
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_CONVEX_HULL_AREA, &(par.ConvexHullArea));
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(par.BoundingRectTop));
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(par.BoundingRectLeft));
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(par.BoundingRectBottom));
						imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(par.BoundingRectRight));
						particles.push_back(par);
					}
					sort(particles.begin(), particles.end(), CompareParticleSizes);

					//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
					//for the reader. Note that the long and short side scores expect a single tote and will not work for a stack of 2 or more totes.
					//Modification of the code to accommodate 2 or more stacked totes is left as an exercise for the reader.
					scores.Trapezoid = TrapezoidScore(particles.at(0));
					SmartDashboard::PutNumber("Trapezoid", scores.Trapezoid);
					scores.LongAspect = LongSideScore(particles.at(0));
					SmartDashboard::PutNumber("Long Aspect", scores.LongAspect);
					scores.ShortAspect = ShortSideScore(particles.at(0));
					SmartDashboard::PutNumber("Short Aspect", scores.ShortAspect);
					scores.AreaToConvexHullArea = ConvexHullAreaScore(particles.at(0));
					SmartDashboard::PutNumber("Convex Hull Area", scores.AreaToConvexHullArea);
					bool isTote = scores.Trapezoid > SCORE_MIN && (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN) && scores.AreaToConvexHullArea > SCORE_MIN;
					bool isLong = scores.LongAspect > scores.ShortAspect;

					//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
					SmartDashboard::PutBoolean("IsTote", isTote);
					SmartDashboard::PutNumber("Distance", computeDistance(binaryFrame, particles.at(0), isLong));
				} else {
					SmartDashboard::PutBoolean("IsTote", false);
				}
			}
			
			float f_throttle = (drive_stick.GetThrottle()+2);
			float f_SpeedAdjust = 2 * f_throttle ;
			float f_CurrentX=drive_stick.GetX();
			float f_CurrentY=drive_stick.GetY();
			float f_CurrentZ =drive_stick.GetZ();
			right_motor.Set(f_CurrentY/f_SpeedAdjust  + f_CurrentX/f_SpeedAdjust + f_CurrentZ/f_SpeedAdjust);
			left_motor.Set(f_CurrentX/f_SpeedAdjust  -  f_CurrentY/f_SpeedAdjust + f_CurrentZ/f_SpeedAdjust);
			up_down_motor.Set(up_down_stick.GetY());
			open_close_motor.Set(up_down_stick.GetX()/2);

			if (drive_stick.GetRawButton(11) == true) {
				yellow = true;
				green = false;
				/*basevalue = 40;
				maxvalue = 60;
				TOTE_HUE_RANGE.minValue = 40;
				TOTE_HUE_RANGE.maxValue = 60;

				TOTE_HUE_RANGE = {basevalue, maxvalue};*/
			}

			else if (drive_stick.GetRawButton(12) == true) {
				green = true;
				yellow = false;
				/*basevalue = 80;
				maxvalue = 120;
				TOTE_HUE_RANGE.minValue = 80;
				TOTE_HUE_RANGE.maxValue = 120;
				TOTE_HUE_RANGE = {basevalue, maxvalue};*/
			}
			//SmartDashboard::PutNumber("basevalue", basevalue);
			//SmartDashboard::PutNumber("maxvalue", maxvalue);
			Wait(0.005);				// wait for a motor update time
		}
        // stop image acquisition
		IMAQdxStopAcquisition(session);
	}
};

START_ROBOT_CLASS(Robot);
