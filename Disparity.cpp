#include "opencv2/opencv.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <stdio.h>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"


using namespace cv;
using namespace std;
using namespace cv::ximgproc;

int     main(int argc, char** argv);
int     detect_object(Mat dispmap, uint x1, uint x2);
int     process(VideoCapture& capture0, VideoCapture& capture1);
Mat     stereocalc(Mat g0, Mat g1);

string  motionCar(int a, int b, int c);
string  IntToStr(int n);
void windowFilters();
void WindowStereoBM();
void windowCamera();
void control();
void filters(Mat img);
void rotation(Mat g0, Mat g1);
Mat linedetect(Mat originalL);
void save_control();
void reload_control();
Mat stereobm(Mat g0, Mat g1);


Mat imgL, imgR; // vstupne obrazky
Mat lenght;    // orezany obrazok
bool first = true;

Mat imgDisparity16S1, imgDisparity16S, depth;
Mat imgDisparity8U1, imgDisparity8U, VisualDisparity, pomocnyfilter;
Mat left_disp, right_disp, filtered_disp;


int menu = 1;

Mat vrch, stred, spodok;
Mat buffer2(Mat image);
Mat *pole = new Mat[3];
Mat b0, b1, b2;

int vmin = 1, vmax = 3, smin = 4, mdip = 15, ndip = 10, sp1 = 50, sp2 = 300; // premenne pre stereosgbm
int dmd = 10, pfc = 10, sur = 30, sws = 10, ssr = 10, sm = 10, bsiz = 13;          //  premenne pre stereosgbm
int DELAY_BLUR = 100, MAX_KERNEL_LENGTH = 0;                      //   premenne pre focus
int l1 = 90, l2 = 180, l3 = 255;                                                      //    premenne pre treshold
int a, b, c;
int lambda = 30000;
int sigma = 2;
int valuedisps = 1;
int LCthresh = 20, alpha = 0, beta = 255;
string filename0, filename1, motion;
double minVal, maxVal;  // identifikacia max a min hodnoty v disparitnej mape
int MAX_BLUR_VALUE = 5, MAX_GAUSS_VALUE = 0, MAX_MEDIAN_VALUE = 0, MAX_BILAT_LENGTH = 0;
int RES_BLUR_VALUE = 5, RES_GAUSS_VALUE = 0, RES_MEDIAN_VALUE = 0, RES_BILAT_LENGTH = 0;
int val1 = 3, val2 = 3, val3 = 7, val4 = 21;
int rotat = 0;

int track, edmin = 61, edmax = 187, ehm = 3, rho = 1, maximka = 255;
int ehm1 = 3, rho1 = 1, maximka1 = 255;
Mat edge, grey, img1, nnn;
int i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;
Mat falseColorsMap;

int iLowH = 159;
int iHighH = 255;

int iLowS = 161;
int iHighS = 255;

int iLowV = 173;
int iHighV = 255;


//argumenty main funkcie
const char* keys =
{
	"{@camera_number01| 1 |camera number}"
	"{@camera_number02| 2 |camera number}"
};/*inicializacne hodnoty*/

int main(int argc, char** argv)
{
	CommandLineParser parser(argc, argv, keys);

	int camL = parser.get<int>(1);
	int camR = parser.get<int>(0);

	control();

	for (;;)
	{
		Mat frame0, frame1;

		VideoCapture capture0(camL);
		VideoCapture capture1(camR);

		capture0.set(CAP_PROP_FRAME_WIDTH, 320);
		capture0.set(CAP_PROP_FRAME_HEIGHT, 240);
		capture0.set(CAP_PROP_FPS, 30);

		capture1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		capture1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
		capture1.set(CAP_PROP_FPS, 30);

		return process(capture0, capture1);
	}
}

int process(VideoCapture& capture0, VideoCapture& capture1)
{
	a, b, c = 0;

	int s = 0;                // pre pocitanie ulozenych obrazkov
	// pre prvotne rozlozenie okien na monitore
	while (1)
	{
		Mat originalL, originalR;
		capture0 >> imgL;
		capture1 >> imgR;
		capture0 >> originalL;
		capture1 >> originalR;
		if (!imgL.empty() && !imgR.empty())
		{

			originalL = linedetect(originalL);

			if (rotat == 1)
			{
				rotation(imgL, imgR);
			}

			filters(imgL);
			filters(imgR);

			if (first == true)
			{
				reload_control();
			}

			VisualDisparity = stereocalc(imgL, imgR);

			filters(VisualDisparity);

			a = detect_object(VisualDisparity, 40, 80);
			b = detect_object(VisualDisparity, 140, 199);
			c = detect_object(VisualDisparity, 203, 280);




			//			bitwise_not(VisualDisparity, depth);
			inRange(VisualDisparity, l1, l2, lenght);

			applyColorMap(VisualDisparity, depth, COLORMAP_JET);
			motion = motionCar(a, b, c);
		}

		//****************zobrazovanie okien*******************//

		putText(originalL, motion, Point2f(20, 20), FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 255), 1, LINE_AA);
		imshow("Rotated L", imgL);
		imshow("Rotated R", imgR);
		imshow("disparity", VisualDisparity);
		imshow("l1,2", lenght);
		imshow("depth", depth);
		imshow("lava kamera", originalL);

		switch (menu)
		{
		case 1:
			windowFilters();
			cvDestroyWindow("StereoBM control");
			cvDestroyWindow("track control");
			break;

		case 2:
			WindowStereoBM();
			cvDestroyWindow("track control");
			cvDestroyWindow("Filters");
			break;

		default:
			windowCamera();
			cvDestroyWindow("StereoBM control");
			cvDestroyWindow("Filters");
		}


		if (first == true)
		{
			moveWindow("Rotated L", 0, 0);
			moveWindow("Rotated R", 325, 0);
			moveWindow("disparity", 650, 0);
			moveWindow("l1,2", 0, 270);
			moveWindow("depth", 650, 270);
			moveWindow("lava kamera", 325, 540);
			moveWindow("prava kamera", 650, 540);
			moveWindow("control", 975, 900);
			resizeWindow("control", 325, 100);
			first = false;
		}

		moveWindow("Filters", 975, 0);
		moveWindow("StereoBM control", 975, 0);
		moveWindow("track control", 975, 0);

		//q************* tlacidla a nejake funkcie**************//
		char key = (char)waitKey(30);
		switch (key)
		{
		case 'q':
		case 'Q':
		case  27: //escape key
			return 0;
		case 's':
			filename0 = "imgR" + IntToStr(s) + ".png";
			filename1 = "imgL" + IntToStr(s) + ".png";
			imwrite(filename0, originalR);
			imwrite(filename1, originalL);
			s++;

		case 'w':
			save_control();
		case 'r':
			reload_control();
			break;

		default:
			break;
		}
	}
	return 0;
}

Mat linedetect(Mat leftImage)
{
	Mat frame = leftImage;
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"


	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 255);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);


	Rect myROI(0, 100, 320, 140);
	Mat lineIm = leftImage(myROI);
	// cvtColor(img, grey, COLOR_BGR2HSV);

	inRange(lineIm, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), grey);
	imshow("free", grey);

	// cvtColor(img,grey,CV_BGR2GRAY);
	//        inRange(grey, 0, track, nnn);
	//  threshold(grey, nnn, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	Canny(grey, edge, edmin, edmax, 3);
	vector<Vec4i> lines;
	HoughLinesP(edge, lines, rho + 1, CV_PI / ehm, maximka, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(lineIm, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
	}


	Mat lineImgL = frame;
	lineIm.copyTo(lineImgL(Rect(0, 100, lineIm.cols, lineIm.rows)));

	imshow("edge", edge);
	if (first == true)
	{
		//       moveWindow("lineImgL", 325, 270);
		moveWindow("edge", 0, 540);
	}
	return leftImage;
}

void rotation(Mat leftFrame, Mat rightFrame)
{

	Size imageSize = leftFrame.size();
	FileStorage fs1("intrinsics.yml", CV_STORAGE_READ);
	Mat  M1, D1, M2, D2;
	fs1["M1"] >> M1;
	fs1["M2"] >> M2;
	fs1["D1"] >> D1;
	fs1["D2"] >> D2;


	FileStorage fs2("extrinsics.yml", CV_STORAGE_READ);
	Mat  R1, P1, R2, P2;
	fs2["R1"] >> R1;
	fs2["R2"] >> R2;
	fs2["P1"] >> P1;
	fs2["P2"] >> P2;

	Mat rmap[2][2];


	Mat img1r, img2r;
	Mat map11, map12, map21, map22;

	Rect ValidRoi0(86, 52, 551, 396);
	Rect ValidRoi1(52, 42, 542, 384);

	initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);

	remap(leftFrame, img1r, map11, map12, INTER_LINEAR);
	remap(rightFrame, img2r, map21, map22, INTER_LINEAR);


	Mat canvasPart = img1r(Rect(320 * 0, 0, 320, 240));
	resize(img1r, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);

	imshow("canvas", canvasPart);


	imgL = img1r;
	imgR = img2r;
}

Mat resultFilters(Mat result)
{
	if (RES_BLUR_VALUE % 2 == 0 && RES_BLUR_VALUE != 0)
		RES_BLUR_VALUE++;
	if (RES_GAUSS_VALUE % 2 == 0 && RES_GAUSS_VALUE != 0)
		RES_GAUSS_VALUE++;
	if (RES_MEDIAN_VALUE % 2 == 0 && RES_MEDIAN_VALUE != 0)
		RES_MEDIAN_VALUE++;
	if (RES_BILAT_LENGTH % 2 == 0 && RES_BILAT_LENGTH != 0)
		RES_BILAT_LENGTH++;
	for (int i = 1; i < RES_BLUR_VALUE; i = i + 2)
	{
		blur(result, result, Size(i, i), Point(-1, -1));
	}

	/// Applying Gaussian blur
	for (int i = 1; i < RES_GAUSS_VALUE; i = i + 2)
	{
		GaussianBlur(result, result, Size(i, i), 0, 0);
	}

	/// Applying Median blur
	for (int i = 1; i < RES_MEDIAN_VALUE; i = i + 2)
	{
		medianBlur(result, result, i);
	}

	return result;
}

Mat stereocalc(Mat g0, Mat g1)
{
	Ptr<DisparityWLSFilter> wls_filter;
	Rect ROI;
	double minVal, maxVal;

	//WindowStereoBM(); /*Trackbars pre stereoBM*/
	if (vmax == 0) vmax = 1;

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(vmin + 1, 16 * vmax, 2 * smin + 1);

	sgbm->setMinDisparity(mdip - 50);
	sgbm->setBlockSize(bsiz + 1);
	sgbm->setP1(sp1 - 10);
	sgbm->setP2(sp2 - 10);
	sgbm->setDisp12MaxDiff(dmd - 21);
	sgbm->setPreFilterCap(pfc - 10);
	sgbm->setUniquenessRatio(sur - 15);
	sgbm->setSpeckleWindowSize(sws - 10);
	sgbm->setSpeckleRange(ssr - 10);
	sgbm->setMode(StereoSGBM::MODE_HH);
	wls_filter = createDisparityWLSFilter(sgbm);
	// g0.convertTo(g0,CV_8UC1,1,0);
	//   g1.convertTo(g1,CV_8UC1,1,0);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(sgbm);
	sgbm->compute(g0, g1, left_disp);
	right_matcher->compute(g1, g0, right_disp);
	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	wls_filter->setLRCthresh(LCthresh);
	wls_filter->filter(left_disp, g0, filtered_disp, right_disp, ROI, g1);

	//  inRange(filtered_disp, l1, l2, left_disp);
	// inRange(right_disp, l1, l2, right_disp);

	normalize(left_disp, imgDisparity8U1, alpha, beta, CV_MINMAX, CV_8U);
	//adveight();
	//	getDisparityVis(imgDisparity8U1, imgDisparity8U1, 8); // pTO
	//  imgDisparity8U1.convertTo(imgDisparity8U1,CV_16S,1,0);
	//minMaxLoc(left_disp, &minVal, &maxVal);
	imgDisparity8U1 = resultFilters(imgDisparity8U1);

	//windowFilters(); /*filtre blur*/
	return (imgDisparity8U1);
}

string motionCar(int a, int b, int c)
{
	if ((a != 1) && (b != 2) && (c != 3))
	{
		motion = "bez prekážky";
	}
	if ((a != 1) && (b != 2) && (c == 3))
	{
		motion = "prekážka vpravo";
	}
	if ((a != 1) && (b == 2) && (c != 3))
	{
		motion = "prekážka v strede";
	}
	if ((a != 1) && (b == 2) && (c == 3))
	{
		motion = "prekážka vpravo a v strede";
	}
	if ((a == 1) && (b != 2) && (c != 3))
	{
		motion = "prekázka vlavo";
	}
	if ((a == 1) && (b != 2) && (c == 3))
	{
		motion = "prekážka vpravo a vlavo";
	}
	if ((a == 1) && (b == 2) && (c != 3))
	{
		motion = "prekážka v strede a vlavo";
	}
	if ((a == 1) && (b == 2) && (c == 3))
	{
		motion = "prekážka vpravo, vlavo a v strede";
	}
	return motion;
}

int detect_object(Mat dispmap, uint x1, uint x2)
{
	Scalar valueD;
	uint i, j, y1 = 1, y2 = 239, pb = 0;
	int prem = 0;

	for (i = x1; i <= x2; i++)
	{
		for (j = y1; j <= y2; j++)
		{
			valueD = dispmap.at<uchar>(Point(i, j));
			if ((valueD.val[0]>l1) && (valueD.val[0]<l2))
			{
				pb++;
			}
		}
	}

	if (pb >= 70)
	{
		if ((i >= 30) && (i <= 85))prem = 1;

		if ((i >= 86) && (i <= 200))prem = 2;

		if ((i >= 201) && (i <= 284))prem = 3;
	}
	return prem;
}

string IntToStr(int n)
{// prevod cisla na retazec znakov, ukladanie obrazkov
	stringstream result;
	result << n;
	return result.str();
}

void filters(Mat img)
{
	//createTrackbar("bilateral", "Filters", &MAX_BILAT_LENGTH, 50, 0);

	if (MAX_BLUR_VALUE % 2 == 0 && MAX_BLUR_VALUE != 0)
		MAX_BLUR_VALUE++;
	if (MAX_GAUSS_VALUE % 2 == 0 && MAX_GAUSS_VALUE != 0)
		MAX_GAUSS_VALUE++;
	if (MAX_MEDIAN_VALUE % 2 == 0 && MAX_MEDIAN_VALUE != 0)
		MAX_MEDIAN_VALUE++;
	if (MAX_BILAT_LENGTH % 2 == 0 && MAX_BILAT_LENGTH != 0)
		MAX_BILAT_LENGTH++;
	for (int i = 1; i < MAX_BLUR_VALUE; i = i + 2)
	{
		blur(img, img, Size(i, i), Point(-1, -1));
	}

	/// Applying Gaussian blur
	for (int i = 1; i < MAX_GAUSS_VALUE; i = i + 2)
	{
		GaussianBlur(img, img, Size(i, i), 0, 0);
	}

	/// Applying Median blur
	for (int i = 1; i < MAX_MEDIAN_VALUE; i = i + 2)
	{
		medianBlur(img, img, i);
	}
	/// Applying Bilateral Filter

	for (int i = 1; i < MAX_BILAT_LENGTH; i = i + 2)
	{
		bilateralFilter(img, img, i, i * 2, i / 2);
	}
}

void windowCamera()
{
	namedWindow("track control", 0);
	createTrackbar("track", "track control", &track, 255, 0);
	createTrackbar("edmin", "track control", &edmin, 255, 0);
	createTrackbar("edmax", "track control", &edmax, 255, 0);
	createTrackbar("nwm", "track control", &ehm, 360, 0);
	createTrackbar("rho", "track control", &rho, 10, 0);
	createTrackbar("maximka", "track control", &maximka, 500, 0);
	createTrackbar("nwm1", "track control", &ehm1, 360, 0);
	createTrackbar("rho1", "track control", &rho1, 10, 0);
	createTrackbar("maximka1", "track control", &maximka1, 500, 0);
}

void WindowStereoBM()
{
	namedWindow("StereoBM control", 0);
	createTrackbar("Vmin", "StereoBM control", &vmin, 99, 0);
	createTrackbar("Vmax", "StereoBM control", &vmax, 15, 0);
	createTrackbar("Smin", "StereoBM control", &smin, 30, 0);
	createTrackbar("mdip", "StereoBM control", &mdip, 99, 0);
	createTrackbar("dmd", "StereoBM control", &dmd, 99, 0);
	createTrackbar("bsiz", "StereoBM control", &bsiz, 99, 0);
	createTrackbar("sp1", "StereoBM control", &sp1, 1000, 0);
	createTrackbar("sp2", "StereoBM control", &sp2, 5000, 0);
	createTrackbar("pfc", "StereoBM control", &pfc, 200, 0);
	createTrackbar("sur", "StereoBM control", &sur, 30, 0);
	createTrackbar("sws", "StereoBM control", &sws, 200, 0);
	createTrackbar("ssr", "StereoBM control", &ssr, 30, 0);
	createTrackbar("alpha", "StereoBM control", &alpha, 300, 0);
	createTrackbar("beta", "StereoBM control", &beta, 300, 0);
	createTrackbar("L1", "StereoBM control", &l1, 255, 0);
	createTrackbar("L2", "StereoBM control", &l2, 255, 0);
	if (l2 <= l1) l2 = l1 + 1;
	if (l3 <= l2) l3 = l2 + 1;

}

void windowFilters()
{
	namedWindow("Filters", 0);
	createTrackbar("blur", "Filters", &MAX_BLUR_VALUE, 50, 0);
	createTrackbar("gauss", "Filters", &MAX_GAUSS_VALUE, 50, 0);
	createTrackbar("median", "Filters", &MAX_MEDIAN_VALUE, 50, 0);
	createTrackbar("resBlur", "Filters", &RES_BLUR_VALUE, 50, 0);
	createTrackbar("resGauss", "Filters", &RES_GAUSS_VALUE, 50, 0);
	createTrackbar("resMedian", "Filters", &RES_MEDIAN_VALUE, 50, 0);
	createTrackbar("Lambda", "Filters", &lambda, 50000, 0);
	createTrackbar("Sigma", "Filters", &sigma, 5, 0);
	createTrackbar("Lcthresh", "Filters", &LCthresh, 100, 0);
	createTrackbar("rotation", "Filters", &rotat, 1, 0);
}

void control()
{
	namedWindow("control", 0);
	createTrackbar("blur", "control", &menu, 2, 0);
}

void save_control()
{
	FileStorage fs("stereobm.yml", FileStorage::WRITE);

	if (fs.isOpened())
	{
		fs << "vmin" << vmin << "vmax" << vmax << "smin" << mdip << "bsiz" << bsiz << "sp1" << sp1 << "sp2" << sp2 << "dmd" << dmd << "pfc" << pfc << "sur" << sur << "sws" << sws << "ssr" << ssr;
		fs.release();
	}
}

void reload_control()
{

	FileStorage fs("stereobm.yml", FileStorage::READ);

	fs["vmin"] >> vmin;
	fs["vmax"] >> vmax;
	fs["smin"] >> smin;
	fs["mdip"] >> mdip;
	fs["bsiz"] >> bsiz;
	fs["sp1"] >> sp1;
	fs["sp2"] >> sp2;
	fs["dmd"] >> dmd;
	fs["pfc"] >> pfc;
	fs["sur"] >> sur;
	fs["sws"] >> sws;
	fs["ssr"] >> ssr;

	fs.release();
}
