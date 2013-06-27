#include "Point.h"

Point::Point(){}

Point::Point(double _x, double _y, double _z){
	x = _x;
	y = _y;
	z = _z;
}

Point::Point(gp_Pnt pnt){
	x = pnt.X();
	y = pnt.Y();
	z = pnt.Z();
}

Point::~Point(void){}

void Point::operator=(Point const& pnt){
	x = pnt.x;
	y = pnt.y;
	z = pnt.z;
}

bool Point::operator==(Point const& other) const{
	if(x == other.x)
		if(y == other.y)
			if(z == other.z)
				return true;
	return false;
}

bool Point::operator!=(Point const& other) const{
	if(x == other.x)
		if(y == other.y)
			if(z == other.z)
				return false;
	return true;
}

gp_Pnt Point::getGP_Pnt(){
	return gp_Pnt(x, y, z);
}


bool Point::cmpNormal(Point pnt){
	if(cmpDbl(x, pnt.x))
		if(cmpDbl(y, pnt.y))
			if(cmpDbl(z, pnt.z))
				return true;
	return false;	
}

// Vergleich für Normale -> -+ oder +- ist auf jeden Fall false
bool Point::cmpDbl(double d1, double d2){
	if(d1 == d2)
		return true;
	if((d1 < 0) && (d2 < 0)){
		d1 *= -1; d2 *= -1;
		double diff;
		if(d2 > d1)
			diff = d2 -d1;
		else
			diff = d1 -d2;
		if(diff < NORMAL_CMP_TOL)
			return true;
	}
	if((d1 > 0) && (d2 > 0)){
		double diff;
		if(d2 > d1)
			diff = d2 -d1;
		else
			diff = d1 -d2;
		if(diff < NORMAL_CMP_TOL)
			return true;
	}
	return false;
}

/*double Point::normalize(double tn){
	if(tn < 0){
		double res = tn;
		res *= -1;
		return res;
	}
	return tn;
}*/


void Point::print() const{
	cout << setprecision(16) << x;
	cout << "/";
	cout << setprecision(16) << y;
	cout << "/";
	cout << setprecision(16) << z;
	cout << endl;
}

void Point::printLOG(fstream * file){
	*file << x;
	*file << "/";
	*file << y;
	*file << "/";
	*file << z;
	*file << endl;
}

/*
Point::Point(const Point& pnt){
	x = pnt.x;
	y = pnt.y;
	z = pnt.z;
}
*/
