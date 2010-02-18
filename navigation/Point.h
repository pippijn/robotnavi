#pragma once

#include <gp_Pnt.hxx>

#include <string>
#include <iostream>
#include <fstream>
using namespace std;

const double NORMAL_CMP_TOL = 0.01;

class Point{
public:
	long double x;
	long double y;
	long double z;
private:
	bool cmpDbl(double d1, double d2);
	//double normalize(double tn);
public:
	Point();
	Point(gp_Pnt pnt);
	Point(double _x, double _y, double _z);
	//Point(const Point& pnt);
	
	~Point(void);

	void operator=(Point const& other);
	bool operator==(Point const& other) const;
	bool operator!=(Point const& other) const;

	gp_Pnt getGP_Pnt();

	bool cmpNormal(Point pnt);
	

	void print() const;
	void printLOG(fstream * file);
};
