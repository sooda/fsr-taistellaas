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
	const float tilt = 3.14; // degree
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

/*
[22:11:20] Â« MulppiÂ» jos T ois [0 0 1], p ois [x y z], v ois [v_x v_y v_z], t ois [t_x t_y 0] ja k ois skalaari
[22:12:05] Â« MulppiÂ» niin T(p+k*v)=0 saa k:n ja p+kv=t saa t:n
[22:14:03] Â« MulppiÂ» k = (-Tp')/(Tv')
[22:14:43] Â« MulppiÂ» eli t = p - (Tp')/(Tv')*v
*/

	Matrix3f R;

//	R = AngleAxisf(tilt, Vector3f::UnitY()) /* * AngleAxisf(pan, Vector3f::UnitZ()) */ ;
	R = AngleAxisf(-M_PI/4, Vector3f::UnitY()) * AngleAxisf(M_PI/4, Vector3f::UnitZ());

	Vector3f T(0, 0, 1); // floor
	Vector3f p(5, 0, 50); // center of the image
	Vector3f v(1, 0, 0); // direction of the view of the camera
	Vector3f t(0,0,0);

	v = R.transpose() * v;
	cout << v << endl;

	t = p - ((T.transpose() * p)/(T.transpose() * v) * v.transpose()).transpose();

	cout << "pallo: " << t << endl;

	float distance = sqrt(t.x() * t.x() + t.y() * t.y());

	cout << "dist: " << distance << endl;

/*
	Vector4f point(0,0,0,1); // origo


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
*/
	return 0;
}
