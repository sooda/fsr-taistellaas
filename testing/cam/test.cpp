#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

using namespace cv;
using namespace Eigen;
using namespace std;

void colorize( int, void* );

Mat src;
Mat dst;

int h = 10;
int s = 50;
int v = 90;

Vector2f getCamRotation()
{
//    float tilt = -M_PI/4;
    float tilt = 0;
    float pan = -M_PI/2;

    Vector2f angles (tilt, pan);

    return angles;

}

Matrix3f getObjectRotation(const float left, const float top)
{
    // image
    const float width = 640;
    const float height = 480;

    // servos
    Vector2f angles = getCamRotation();
    const float tilt = angles(0);
    const float pan = angles(1);

    const float fov = 50.0 / 360.0 * M_PI;

    // yksi pikseli vastaa radiaaneja (pystysuunnassa) (pixel per degree) (voidaanko käyttää myös vaakasuunnassa?)
    const float ppd = fov/height;

    // kohteen etäisyys keskipisteestä
    float ydist = height/2 - top;
    float xdist = width/2 - left;

    // tarkoittaa kulmina xangl radiaania
    float yangl = ppd*ydist;
    float xangl = ppd*xdist;

    // tällöin kulma
    float ya = tilt + yangl;
    float xa = pan + xangl;


    Matrix3f rotation;

    rotation = AngleAxisf(ya, Vector3f::UnitY()) * AngleAxisf(xa, Vector3f::UnitZ());

    return rotation;

}

void printDist(const int left, const int top)
{

    Matrix3f R = getObjectRotation(left, top);

    Vector3f T(0, 0, 1); // floor
    Vector3f p(5, 0, 50); // position of the camera
    Vector3f v(1, 0, 0); // direction of the view of the camera
    Vector3f t(0,0,0);

    v = R.transpose() * v;
    cout << v << endl;

    t = p - ((T.transpose() * p)/(T.transpose() * v) * v.transpose()).transpose();

    cout << t << endl;

    float distance = sqrt(t.x() * t.x() + t.y() * t.y());

    cout << "dist: " << distance << endl;

}

int main(int argc, char** argv)
{

	src = imread ( argv[1], 1 );
	cvtColor(src, dst, CV_BGR2HSV);


	/* Create Window and Trackbar */
/*
	namedWindow( win, CV_WINDOW_AUTOSIZE );
	createTrackbar( " H: (0-h)", win, &h, 255, colorize );
	createTrackbar( " S: (s-255)", win, &s, 255, colorize );
	createTrackbar( " V: (v-255)", win, &v, 255, colorize );
*/
	colorize( 0, 0 );

	waitKey(0);
	return 0;

}

void colorize (int, void*)
{

//        Mat src, dst;

//	src = imread ( argv[1], 1 );
	cvtColor(src, dst, CV_BGR2HSV);

	// tresholding
//	inRange(dst, Scalar(0, 160, 160), Scalar(10, 255, 255), dst);
	inRange(dst, Scalar(0, s, v), Scalar(h, 255, 255), dst);

	// dilation
	int dilation_size = 2;
	Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
	dilate( dst, dst, element );

	imshow("dilation", dst);

	// contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for(int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
	{
//		double area = contourArea(contours[idx]);
		Scalar color( rand()&255, rand()&255, rand()&255 );
//		if (area < 100) continue;
//		if (area > 1000) color = Scalar(0,0,255);

		Point2f center;
		float radius;
		minEnclosingCircle(contours[idx], center, radius);

		if (radius < 9) continue;
		if (center.x < 3 || center.y < 3) continue;
		std::cout << idx << ": " << center << " " << radius << std::endl;
		printDist(center.x, center.y);

//		drawContours( src, contours, idx, color, CV_FILLED, 8, hierarchy );
		circle( src, center, (int)radius * 1.2, color, 2);
	}

//	imshow( "origin", src);
//        imshow( "padam", dst);
	imshow("src", src);

//	waitKey(0);


}

