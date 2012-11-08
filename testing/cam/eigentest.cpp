#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main (int argc, char** argv)
{
/*
oletetaan, että löydetään 800x600 kuvasta pallo, keskipisteeltään kohdassa (400,300)
*/

/*
[22:11:20] « Mulppi» jos T ois [0 0 1], p ois [x y z], v ois [v_x v_y v_z], t ois [t_x t_y 0] ja k ois skalaari
[22:12:05] « Mulppi» niin T(p+k*v)=0 saa k:n ja p+kv=t saa t:n
[22:14:03] « Mulppi» k = (-Tp')/(Tv')
[22:14:43] « Mulppi» eli t = p - (Tp')/(Tv')*v
*/

	// image
	const float width = 800;
	const float height = 600;

	// servos
	const float tilt = -M_PI/4; // degree
	const float pan = 0.0;

	const float fov = 50.0 / 360.0 * M_PI;

	// object
	const int left = 400;
	const int top = 300;

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


	Matrix3f R;

	R = AngleAxisf(ya, Vector3f::UnitY()) * AngleAxisf(xa, Vector3f::UnitZ());

	Vector3f T(0, 0, 1); // floor
//	Vector3f p(5, 0, 50); // position of the camera
	Vector3f p(0, 0, 100); // position of the camera
	Vector3f v(1, 0, 0); // direction of the view of the camera
	Vector3f t(0,0,0);

	v = R.transpose() * v;
	cout << v << endl;

	t = p - ((T.transpose() * p)/(T.transpose() * v) * v.transpose()).transpose();

	cout << "pallo: " << t << endl;

	float distance = sqrt(t.x() * t.x() + t.y() * t.y());

	cout << "dist: " << distance << endl;

	return 0;
}
