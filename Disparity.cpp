#include "opencv2/opencv.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <stdio.h>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc/edge_filter.hpp"

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

int     main(int argc, char** argv);
int     detect_object(Mat dispmap, uint x1, uint x2);
int     process(VideoCapture& capture0, VideoCapture& capture1);
Mat     stereocalc(Mat g0, Mat g1);
string  motionCar(int a, int b, int c);
string  IntToStr(int n);

Mat g0, g1, g3;
Mat imgDisparity16S1, imgDisparity16S, depth;
Mat imgDisparity8U1, imgDisparity8U, VisualDisparity, pomocnyfilter;
Mat left_disp, right_disp, filtered_disp;
Mat lenght1, lenght2, lenght3;

int vmin = 1, vmax = 3, smin = 0, mdip = 15, ndip = 10, sp1 = 50, sp2 = 300; // premenne pre stereosgbm
int dmd = 10, pfc = 10, sur = 30, sws = 10, ssr = 10, sm = 10, bsiz = 13;          //  premenne pre stereosgbm
int DELAY_BLUR = 100, MAX_KERNEL_LENGTH = 11;                      //   premenne pre focus
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

int i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;
Mat falseColorsMap;
//argumenty main funkcie
const char* keys =
{
	"{@camera_number01| 1 |camera number}"
	"{@camera_number02| 2 |camera number}"
};/*inicializacne hodnoty*/

int main(int argc, char** argv)
{
	//rozdelenie vstupnych argumentov, lava a prava kamera
	CommandLineParser parser(argc, argv, keys);
	int camL = parser.get<int>(1);
	int camR = parser.get<int>(0);

	for (;;)
	{
		Mat frame0, frame1;

		VideoCapture capture0(camL);
		VideoCapture capture1(camR);

		capture0.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		capture0.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
		capture1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		capture1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

		return process(capture0, capture1);
	}
}

void WindowStereoBM()
{
	namedWindow("StereoBM control", 0);
	createTrackbar("Vmin", "StereoBM control", &vmin, 99, 0);
	createTrackbar("Vmax", "StereoBM control", &vmax, 99, 0);
	createTrackbar("Smin", "StereoBM control", &smin, 30, 0);
	createTrackbar("mdip", "StereoBM control", &mdip, 99, 0);
	createTrackbar("dmd", "StereoBM control", &dmd, 99, 0);
	createTrackbar("bsiz", "StereoBM control", &bsiz, 99, 0);
	createTrackbar("sp1", "StereoBM control", &sp1, 1000, 0);
	createTrackbar("sp2", "StereoBM control", &sp2, 2000, 0);
	createTrackbar("pfc", "StereoBM control", &pfc, 200, 0);
	createTrackbar("sur", "StereoBM control", &sur, 30, 0);
	createTrackbar("sws", "StereoBM control", &sws, 200, 0);
	createTrackbar("ssr", "StereoBM control", &ssr, 30, 0);
	createTrackbar("sm", "StereoBM control", &sm, 640, 0);
	createTrackbar("alpha", "StereoBM control", &alpha, 300, 0);
	createTrackbar("beta", "StereoBM control", &beta, 300, 0);
	createTrackbar("L1", "StereoBM control", &l1, 255, 0);
	createTrackbar("L2", "StereoBM control", &l2, 255, 0);
	createTrackbar("L3", "StereoBM control", &l3, 255, 0);
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

void filters(Mat imgL, Mat imgR)
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
		blur(imgL, imgL, Size(i, i), Point(-1, -1));
		blur(imgR, imgR, Size(i, i), Point(-1, -1));
	}

	/// Applying Gaussian blur
	for (int i = 1; i < MAX_GAUSS_VALUE; i = i + 2)
	{
		GaussianBlur(imgL, imgL, Size(i, i), 0, 0);
		GaussianBlur(imgR, imgR, Size(i, i), 0, 0);
	}

	/// Applying Median blur
	for (int i = 1; i < MAX_MEDIAN_VALUE; i = i + 2)
	{
		medianBlur(imgL, imgL, i);
		medianBlur(imgR, imgR, i);
	}
	/// Applying Bilateral Filter

	for (int i = 1; i < MAX_BILAT_LENGTH; i = i + 2)
	{
		bilateralFilter(imgL, imgL, i, i * 2, i / 2);
		bilateralFilter(imgL, imgL, i, i * 2, i / 2);
	}

}


Mat stereocalc(Mat g0, Mat g1)
{
	Ptr<DisparityWLSFilter> wls_filter;
	Rect ROI;
	double minVal, maxVal;
	WindowStereoBM(); /*Trackbars pre stereoBM*/

	if (l2 <= l1) l2 = l1 + 1;
	if (l3 <= l2) l3 = l2 + 1;
	if (vmax == 0) vmax = 1;

	/*StereBM*/
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(vmin + 1, 16 * vmax, 2 * smin + 1);
	
	sgbm->setMinDisparity(mdip - 20);
	sgbm->setBlockSize(bsiz - 10);
	sgbm->setP1(sp1 - 10);
	sgbm->setP2(sp2 - 10);
	sgbm->setDisp12MaxDiff(dmd - 21);
	sgbm->setPreFilterCap(pfc - 10);
	sgbm->setUniquenessRatio(sur - 15);
	sgbm->setSpeckleWindowSize(sws - 10);
	sgbm->setSpeckleRange(ssr - 10);
	sgbm->setMode(StereoSGBM::MODE_HH);
	wls_filter = createDisparityWLSFilter(sgbm);

	Ptr<StereoMatcher> right_matcher = createRightMatcher(sgbm);
	sgbm->compute(g0, g1, left_disp);
	right_matcher->compute(g1, g0, right_disp);

	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	wls_filter->setLRCthresh(LCthresh);
	wls_filter->filter(left_disp, g0, filtered_disp, right_disp, ROI, g1);
	getDisparityVis(filtered_disp, imgDisparity8U1, 8); // pTO

	minMaxLoc(left_disp, &minVal, &maxVal);
	imgDisparity8U1 = resultFilters(imgDisparity8U1);

	windowFilters(); /*filtre blur*/
	return (imgDisparity8U1);
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



int process(VideoCapture& capture0, VideoCapture& capture1)
{
	a,b,c = 0;

	int s = 0;                // pre pocitanie ulozenych obrazkov
	bool first = true;        // pre prvotne rozlozenie okien na monitore
	while (1)
	{
		capture0 >> g0;
		capture1 >> g1;
		//***********kontrola framov, ci nie su prazdne*********//
		if (!g0.empty() && !g1.empty())
		{
			//******************** rgb to gray**********************//
			cvtColor(g0, g0, CV_BGR2GRAY);
			cvtColor(g1, g1, CV_BGR2GRAY);
			//***********kontrola framov, ci nie su prazdne************//
			//****************** cyklus median bluru******************//
			filters(g0, g1);
			//******************** stereoSGBM***********************//
			VisualDisparity = stereocalc(g0, g1);

			//******************** vypocet *************************//

			a = detect_object(VisualDisparity, 40, 80);
			b = detect_object(VisualDisparity, 140, 199);
			c = detect_object(VisualDisparity, 203, 280);

			//inRange(VisualDisparity, Scalar(50), Scalar(l1), lenght1);
			inRange(VisualDisparity, 50, l1, lenght1);
			inRange(VisualDisparity, l1, l2, lenght2);
			inRange(VisualDisparity, l2, l3, lenght3);

			bitwise_not(VisualDisparity, depth);
			applyColorMap(VisualDisparity, depth, COLORMAP_JET);
			motion = motionCar(a, b, c);
		}

		//****************zobrazovanie okien*******************//

		putText(g1, motion, Point2f(20, 20), FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 255), 1, LINE_AA);
		imshow("Lava kamera", g0);
		imshow("Prava kamera", g1);
		imshow("disparity", VisualDisparity);
		imshow("0,l1", lenght1);
		imshow("l1,2", lenght2);
		imshow("l2,3", lenght3);
		imshow("depth", depth);

		if (first == true)
		{
			moveWindow("Lava kamera", 0, 0);
			moveWindow("Prava kamera", 325, 0);
			moveWindow("disparity", 650, 0);
			moveWindow("0,l1", 0, 270);
			moveWindow("l1,2", 325, 270);
			moveWindow("l2,3", 650, 270);
			moveWindow("depth", 0, 540);
			moveWindow("Filters", 325, 540);
			moveWindow("StereoBM control", 1200, 0);
			first = false;
		}
		//************* tlacidla a nejake funkcie**************//
		char key = (char)waitKey(30);
		switch (key)
		{
		case 'q':
		case 'Q':
		case  27: //escape key
			return 0;
		case 's':
			filename0 = "imgR" + IntToStr(s) + ".jpg";
			filename1 = "imgL" + IntToStr(s) + ".jpg";
			imwrite(filename0, g0);
			imwrite(filename1, g1);
			s++;
		default:
			break;
		}
	}
	return 0;
}

string IntToStr(int n)
{// prevod cisla na retazec znakov, ukladanie obrazkov
	stringstream result;
	result << n;
	return result.str();
}


