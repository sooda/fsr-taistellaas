#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main ()
{
/*
oletetaan, että löydetään 800x600 kuvasta pallo, keskipisteeltään kohdassa (400,300)

saadaan tilt ja pan kulmat: (-0.5pi, 0.5pi) vai asteina??
- tilt = 0.3*pi = 0.94
- pan  = -0.2*pi = -0.63

*/

	// image
	const float height = 600;
	const float width = 800;

	// servos
	const float tilt = 20; // degree
	const float pan = 0.0;

	const float fov = 50;
	
	// object
	const int left = 400;
	const int top = 300;

	// yksi pikseli vastaa asteita (pystysuunnassa) (pixel per degree)
	const float ppd = fov/height;

	// kohteen etäisyys keskipisteestä (pystysuunnassa)
	float xdist = height/2 - top;

	// tarkoittaa kulmina xangl astetta
	float xangl = ppd*xdist;

	// tällöin kulma pystysuunnassa
	float xa = tilt - xangl;
	
	// etäisyys kohteeseen
	float dist = 50 / tan(xa/360*2*3.14) +5;
	
//	cout << width << "x" << height << " kuvassa kohdassa (" << left << "," << top << ") loytyvaan kohteeseen etaisuutta: " << dist << endl;
	
	
	/* matriisisäätö */
	
	Vector4f point(0,0,0,1); // origo
	
	Affine3f R, T1, T2, T3;
	
	R = AngleAxisf(tilt, Vector3f::UnitZ()) * AngleAxisf(pan, Vector3f::UnitY());

	T1 = Translation3f(5, 50, 0);
	
	point = T1 * point;
	point = R * point;
	
	// kameran paikkavektori
	Vector3f = point.topRightCorner<1,3>();
	
	// lattiatasomatriisi
	Matrix3f plane;
	
//	cout << "point:\n" << point << endl;

	point = R.inverse() * point;
//	cout << "inversed point:\n" << point << endl;
//	cout << "top rows: \n" << point.topRows(3) << endl;
	
	return 0;
}