#pragma once
#include "Point.h"
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <GC_MakeSegment.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <TopoDS_Shape.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GC_MakeSegment.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>

class MathVector{
public:
	double x, y, z;

private:
	void tol(gp_Pnt &pnt)const;
	double tol(double value)const;
	double gradToRadian(double grad) const;
public:
	MathVector(void);
	MathVector(double _x, double _y, double _z);
	MathVector(Point from, Point to);
	~MathVector(void);

	MathVector copy() const;
	MathVector multiply(MathVector const& other) const;
	MathVector add(MathVector const& other) const;
	MathVector sub(MathVector const& other) const;
	MathVector unity() const;
	MathVector cross(MathVector const& other) const;
        gp_Pnt getGP_Pnt() const;
	double dotProduct(MathVector const& other) const;
	double norm() const;
	MathVector getOrthogonalVector()const;

	void operator=(const MathVector vec);
	bool operator==(const MathVector other);
	bool operator!=(const MathVector other);

	double angleTo(MathVector const& other) const;

	TopoDS_Edge getTopoDS_Edge();
};
