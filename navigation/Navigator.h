#pragma once

#include <Geom_BSplineCurve.hxx>
#include <gp_Pnt.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepOffsetAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepTools.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <TopoDS_Edge.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomConvert.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Circ.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <GC_MakeCircle.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <gp_Lin.hxx>
#include <BRepExtrema_DistShapeShape.hxx>

#include "MathVector.h"
#include <iostream>

#include <vector>
using namespace std;

enum DIRECTION{
	LEFT, RIGHT, STRAIGHT
};

struct WAYROT{
	double length;
	double angle;
	DIRECTION dir;

	void print();
};

class Navigator{
private:
	vector<gp_Pnt> pnts;
	Handle(Geom_BSplineCurve) curve;
	vector<TopoDS_Face> targets;
	TopoDS_Face final;
	
	int currentTargetPosition;	
	double stepLength;
	bool initialized;
	bool blocked;

	struct handler
	{
		void (*fun)(void* self, int current, int count);
		void* ob;
	} handler;

private:
	TopoDS_Shape toShape(TopoDS_Edge edge);
	TopoDS_Shape toShape(TopoDS_Face face);

private:
	double getProjectedPosition(gp_Pnt const& toProject) const;

	MathVector getTangentStart(Handle(Geom_BSplineCurve) bspline) const;
	MathVector getTangentEnd(Handle(Geom_BSplineCurve) bspline) const;
	vector<Handle(Geom_BSplineCurve)> splitCurve(double segLength) const;

	DIRECTION getDirection(MathVector before, MathVector now) const;

	int getVecDir(MathVector vec) const;
	bool isLeft(int refDir, int checkDir) const;
	bool isRight(int refDir, int checkDir) const;

	void initTargetFaces(vector<gp_Pnt> const& trgts);
	bool joinedTarget(gp_Pnt const& pos);
	bool joinedFinal(gp_Pnt const& pos);
	bool nextTargetReached(gp_Pnt const& pos) const;

public:
	Navigator(double _stepLength);
	~Navigator(void);

	void initialize(vector<gp_Pnt> const& targets);

	MathVector getDestination(gp_Pnt const& position);
	gp_Pnt getNextTarget();
#if 0
	void setBlockade();
#endif
	
	vector<WAYROT> getRotationSegments(double nmbSegs) const;

	double getCurveLength() const;

	void set_handler(void (*fun)(void* self, int current, int count), void* ob)
	{
		handler.fun = fun;
		handler.ob = ob;
	}

	// DEBUG //
	TopoDS_Edge getEdge() const;
	void printCurve(string const& filepath) const;
	///////////
};
