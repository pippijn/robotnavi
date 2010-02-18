#include "Navigator.h"

/////////////////////////////////////////////////////
void WAYROT::print(){
	cout << "--- WAYROT ---" << endl;
	cout << "length: " << length << endl;
	cout << "angle: " << angle << endl;
	cout << "direction: ";
	if(dir == LEFT){
		cout << "LEFT" << endl;
	}else if(dir == RIGHT){
		cout << "RIGHT" << endl;
	}else{
		cout << "STRAIGHT" << endl;
	}
	cout << "--------------" << endl;
}
/////////////////////////////////////////////////////

TopoDS_Shape Navigator::toShape(TopoDS_Edge edge){
	BRepOffsetAPI_Sewing sew(1e-5);
	sew.Add(edge);
	sew.Perform();
	return sew.SewedShape();
}
TopoDS_Shape Navigator::toShape(TopoDS_Face face){
	BRepOffsetAPI_Sewing sew(1e-5);
	sew.Add(face);
	sew.Perform();
	return sew.SewedShape();
}


Navigator::Navigator(double _stepLength){
	stepLength = _stepLength;
	initialized = false;
}

Navigator::~Navigator(void){}

void Navigator::initialize(vector<gp_Pnt> const& targets){
	pnts = targets;
	
	vector<gp_Pnt>::const_iterator it, et;
	it = pnts.begin(); et = pnts.end();
	Handle(TColgp_HArray1OfPnt) interpolationPoints = new TColgp_HArray1OfPnt(1, pnts.size());
	int i=1;
	while(it != et){
		interpolationPoints->SetValue(i, *it);
		++i;
		++it;
	}

	GeomAPI_Interpolate * interpolation = new GeomAPI_Interpolate(interpolationPoints, Standard_False, Precision::Approximation());
	interpolation->Perform();

	curve = interpolation->Curve();
	currentTargetPosition = 0;

	initTargetFaces(targets);
	blocked = false;

	initialized = true;
}

void Navigator::printCurve(string const& filepath) const{
	if(pnts.empty()){
		cout << "No points defined!" << endl;
		return;
	}
	TopoDS_Shape tmp;

	BRepOffsetAPI_Sewing sew(1e-5);
	sew.Add(BRepBuilderAPI_MakeEdge(curve));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(-20000,0,0), gp_Pnt(20000,0,0))));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,-30000,0), gp_Pnt(0,30000,0))));
	sew.Perform();
	tmp = sew.SewedShape();

	BRepTools::Write(tmp, filepath.c_str());
	cout << "BRep exported to " << filepath << endl;
}

TopoDS_Edge Navigator::getEdge() const{
	return BRepBuilderAPI_MakeEdge(curve);
}

double Navigator::getProjectedPosition(gp_Pnt const& toProject) const{
	gp_Pnt proj(0,0,0);
	double pos;
	ShapeAnalysis_Curve analysis;
	analysis.Project(
		curve,
		toProject,
		1e-5,
		proj,
		pos,
		true
	);

#if 0
	// DEBUG //
	double dbg = pos;
	dbg += stepLength;
	gp_Pnt targetPoint(0,0,0);
	curve->D0(dbg, targetPoint);

	TopoDS_Shape tmp;
	BRepOffsetAPI_Sewing sew(1e-5);
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(-20,0,0), gp_Pnt(20,0,0))));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,-30,0), gp_Pnt(0,30,0))));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,0,0), toProject)));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,0,0), proj)));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(toProject, targetPoint)));
	sew.Add(getEdge());
	sew.Perform();
	tmp = sew.SewedShape();

	BRepTools::Write(tmp, "c:\\data\\debug.brep");
	///////////
#endif

	return pos;
}

MathVector Navigator::getTangentStart(Handle(Geom_BSplineCurve) bspline) const{
	gp_Pnt start_pole2 = bspline->Pole(2);
	return MathVector(Point(bspline->StartPoint()), Point(start_pole2));
}

MathVector Navigator::getTangentEnd(Handle(Geom_BSplineCurve) bspline) const{
	gp_Pnt end_pole1 = bspline->Pole(bspline->NbPoles() - 1);
	return MathVector(Point(bspline->EndPoint()), Point(end_pole1));
}

vector<Handle(Geom_BSplineCurve)> Navigator::splitCurve(double segLength) const{
	vector<Handle(Geom_BSplineCurve)> res;
	GeomAdaptor_Curve gac(curve);
	double u = 0;
	double u_next;
	double step = 0;
	double totalLength = getCurveLength();
	gp_Pnt gp_pp(0,0,0);

	while(step < totalLength){
		step += segLength;

		if(step > totalLength){
			step = totalLength;
		}

		GCPnts_AbscissaPoint absc(gac, step, 0);
		gac.D0(absc.Parameter(), gp_pp);
		u_next = getProjectedPosition(gp_pp);

		res.push_back(GeomConvert::SplitBSplineCurve(curve, u, u_next, false));
	
		u = u_next;
	}

#if 1
	// DEBUG //
	BRepOffsetAPI_Sewing sew(1e-5); gp_Pnt reference(-20,15,0);
	vector<Handle(Geom_BSplineCurve)>::const_iterator it, et;
	it = res.begin(); et = res.end();
	while(it != et){
		sew.Add(BRepBuilderAPI_MakeEdge(*it));
		sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(-20,0,0), gp_Pnt(20,0,0))));
		sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,-30,0), gp_Pnt(0,30,0))));
		sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(reference, (*it)->StartPoint())));
		sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(reference, (*it)->EndPoint())));
		sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment((*it)->StartPoint(), (*it)->EndPoint())));
		++it;
	}
	sew.Add(getEdge());
	sew.Perform();
	BRepTools::Write(sew.SewedShape(), "c:\\data\\split.brep");
	///////////
#endif

	return res;
}

DIRECTION Navigator::getDirection(MathVector before, MathVector now) const{
	if(before.angleTo(now) <= 1/*1e-5*/)
		return STRAIGHT;
	int dirB = getVecDir(before);
	int dirN = getVecDir(now);

	cout << "---" << endl;
	cout << dirB << endl;
	cout << dirN << endl;

	if(isLeft(dirB, dirN)){
		return LEFT;
	}if(isRight(dirB, dirN)){
		return RIGHT;
	}else{
		MathVector xAxisP(1,0,0);	// Positive X-Achse
		MathVector xAxisN(-1,0,0);	// Negative X-Achse
		double angleB, angleN;
		if((dirB == 3 || dirB == 7) && (dirN == 3 || dirN == 7)){	// geprüft wird immer der Winkel zur
			if(before.x > 0){										// X-Achse mit unterschiedlichen 
				angleB = xAxisP.angleTo(before);					// Behandlungen
				if(now.x > 0){
					angleN = xAxisP.angleTo(now);
					if(angleN > angleB)
						return RIGHT;
					return LEFT;
				}else if(now.x < 0){
					angleN = xAxisN.angleTo(now);
					if(angleN > angleB)
						return LEFT;
					return RIGHT;
				}else{
					throw -1;
				}
			}else if(before.x < 0){
				angleB = xAxisN.angleTo(before);
				if(now.x > 0){
					angleN = xAxisP.angleTo(now);
					if(angleN > angleB)
						return RIGHT;
					return LEFT;
				}else if(now.x < 0){
					angleN = xAxisN.angleTo(now);
					if(angleN > angleB)
						return RIGHT;
					return LEFT;
				}else{
					throw -1;
				}
			}else{
				throw -1;
			}
		}else{
			if(before.x > 0){
				angleB = xAxisP.angleTo(before);
				if(now.x > 0){
					angleN = xAxisP.angleTo(now);
					if(angleN > angleB)
						return LEFT;
					return RIGHT;
				}else if(now.x < 0){
					angleN = xAxisN.angleTo(now);
					if(angleN > angleB)
						return RIGHT;
					return LEFT;
				}else{
					throw -1;
				}
			}else if(before.x < 0){
				angleB = xAxisN.angleTo(before);
				if(now.x > 0){
					angleN = xAxisP.angleTo(now);
					if(angleN > angleB)
						return LEFT;
					return RIGHT;
				}else if(now.x < 0){
					angleN = xAxisN.angleTo(now);
					if(angleN > angleB)
						return RIGHT;
					return LEFT;
				}else{
					throw -1;
				}
			}else{
				throw -1;
			}
		}		
	}
	throw -2;
}

int Navigator::getVecDir(MathVector vec) const{
	if(vec.x > 0){
		if(vec.y > 0){
			return 1;
		}else if(vec.y < 0){
			return 3;
		}else{
			return 2;
		}
	}else if(vec.x < 0){
		if(vec.y > 0){
			return 7;
		}else if(vec.y < 0){
			return 5;
		}else{
			return 6;
		}
	}else{
		if(vec.y > 0){
			return 8;
		}else if(vec.y < 0){
			return 4;
		}else{
			throw -1;
		}
	}
}

bool Navigator::isLeft(int refDir, int checkDir) const{
	switch(refDir){
		case 1:
			if(checkDir == 8 || checkDir == 7 || checkDir == 6)
				return true;
			break;
		case 2:
			if(checkDir == 1 || checkDir == 8 || checkDir == 7 || checkDir == 6)
				return true;
			break;
		case 3:
			if(checkDir == 2 || checkDir == 1 || checkDir == 8)
				return true;
			break;
		case 4:
			if(checkDir == 3 || checkDir == 2 || checkDir == 1 || checkDir == 8)
				return true;
			break;
		case 5:
			if(checkDir == 4 || checkDir == 3 || checkDir == 2)
				return true;
			break;
		case 6:
			if(checkDir == 5 || checkDir == 4 || checkDir == 3 || checkDir == 2)
				return true;
			break;
		case 7:
			if(checkDir == 6 || checkDir == 5 || checkDir == 4)
				return true;
			break;
		case 8:
			if(checkDir == 7 || checkDir == 6 || checkDir == 5 || checkDir == 4)
				return true;
			break;
		default:
			throw;
	}
	return false;
}

bool Navigator::isRight(int refDir, int checkDir) const{
	switch(refDir){
		case 1:
			if(checkDir == 2 || checkDir == 3 || checkDir == 4)
				return true;
			break;
		case 2:
			if(checkDir == 3 || checkDir == 4 || checkDir == 5 || checkDir == 6)
				return true;
			break;
		case 3:
			if(checkDir == 4 || checkDir == 5 || checkDir == 6)
				return true;
			break;
		case 4:
			if(checkDir == 5 || checkDir == 6 || checkDir == 7 || checkDir == 8)
				return true;
			break;
		case 5:
			if(checkDir == 6 || checkDir == 7 || checkDir == 8)
				return true;
			break;
		case 6:
			if(checkDir == 7 || checkDir == 8 || checkDir == 1 || checkDir == 2)
				return true;
			break;
		case 7:
			if(checkDir == 8 || checkDir == 1 || checkDir == 2)
				return true;
			break;
		case 8:
			if(checkDir == 7 || checkDir == 6 || checkDir == 5 || checkDir == 4)
				return true;
			break;
		default:
			throw;
	}
	return false;
}

void Navigator::initTargetFaces(vector<gp_Pnt> const& trgts){
	vector<gp_Pnt>::const_iterator it, et, bet;
	it = trgts.begin(), et = trgts.end(); bet = et; --bet;
	while(it != et){
		gp_Circ circle(gp_Ax2(*it, gp_Dir(0,0,1)), 25);
		Handle(Geom_Circle) girc = GC_MakeCircle(circle).Value();
		TopoDS_Edge circle_edge = BRepBuilderAPI_MakeEdge( girc );
		TopoDS_Wire circle_wire = BRepBuilderAPI_MakeWire( circle_edge );
		targets.push_back(BRepBuilderAPI_MakeFace(circle_wire));

		if(it == bet){
			final = BRepBuilderAPI_MakeFace(circle_wire); 
		}
		++it;
	}
	// DEBUG //
	BRepOffsetAPI_Sewing sew(1e-5);
	sew.Add(BRepBuilderAPI_MakeEdge(curve));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(-20000,0,0), gp_Pnt(20000,0,0))));
	sew.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,-30000,0), gp_Pnt(0,30000,0))));

	vector<TopoDS_Face>::iterator fit, fet;
	fit = targets.begin(); fet = targets.end();
	while(fit != fet){
		sew.Add(*fit);
		++fit;
	}

	sew.Perform();

	BRepTools::Write(sew.SewedShape(), "c:\\data\\circleCheck.brep");
	///////////
}

bool Navigator::joinedTarget(gp_Pnt const& pos){
	//gp_Lin linToCheck(gp_Pnt(pos.X(), pos.Y(), -10), 
	//	gp_Dir( gp_Vec( pos.X(), pos.Y(), 10 ) ) 
	//);
	TopoDS_Edge linToCheck = BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(pos.X(), pos.Y(), -10), gp_Pnt(pos.X(), pos.Y(), 10)));
	vector<TopoDS_Face>::const_iterator it, et;
	it = targets.begin(); et = targets.end();
	int cnt = 0;
	while(it != et){
		//IntCurvesFace_Intersector checker(*it, 1e-5);
		BRepExtrema_DistShapeShape distanceReader(toShape(*it), toShape(linToCheck));
		try{
			//checker.Perform(linToCheck, -999999, +999999);
			
		}catch(...){
			cout << "Error in >joinedTarget(gp_Pnt pos)< - intersection failed" << endl;
			throw -1;
		}
		//if(checker.NbPnt() > 0){
		if(distanceReader.Value() < 50){
			currentTargetPosition = cnt;
			return true;
		}
		++cnt;
		++it;
	}
	return false;
}

bool Navigator::nextTargetReached(gp_Pnt const& pos) const{
	gp_Lin linToCheck(gp_Pnt(pos.X(), pos.Y(), -10), 
		gp_Dir( gp_Vec( pos.X(), pos.Y(), 10 ) ) 
	);
	vector<TopoDS_Face>::const_iterator it, et;
	it = targets.begin(); et = targets.end();
	int cnt = 0;
	while(it != et){
		IntCurvesFace_Intersector checker(*it, 1e-5);
		try{
			checker.Perform(linToCheck, -999999, +999999);
		}catch(...){
			cout << "Error in >joinedTarget(gp_Pnt pos)< - intersection failed" << endl;
			throw -1;
		}
		if(checker.NbPnt() > 0){
			if(cnt > currentTargetPosition)
				return true;
		}
		++cnt;
		++it;
	}
	return false;
}

MathVector Navigator::getDestination(gp_Pnt const& position){
#if 0
		if(!blocked){
#endif
			if (joinedTarget(position))
                                handler.fun (handler.ob, currentTargetPosition, targets.size () - 1);
			if(!initialized){
				cout << "Navigator needs to be initialized!" << endl;
				throw 1;
			}
			double pos = getProjectedPosition(position);
			pos += stepLength;

			gp_Pnt targetPoint(0,0,0);
			curve->D0(pos, targetPoint);

			return MathVector(Point(position), Point(targetPoint));
#if 0
		}else{
			if(nextTargetReached(position)){
				++currentTargetPosition;
				blocked = false;
				return getDestination(position);
			}
		}
#endif
}

gp_Pnt Navigator::getNextTarget(){
	return pnts[currentTargetPosition+1];
}

#if 0
void Navigator::setBlockade(){
	blocked = true;
}
#endif

/**
*	return -1 wenn nicht initialisiert.
**/
double Navigator::getCurveLength() const{
	if(!initialized){
		cout << "Navigator needs to be initialized!" << endl;
		return -1;
	}
	GeomAdaptor_Curve gac(curve);
	return GCPnts_AbscissaPoint::Length(gac);
}

vector<WAYROT> Navigator::getRotationSegments(double nmbSegs) const{
	if(!initialized){
		cout << "Navigator needs to be initialized!" << endl;
		throw 1;
	}

	vector<WAYROT> res;
	double segLength = getCurveLength() / nmbSegs;
	vector<Handle(Geom_BSplineCurve)> segments = splitCurve(segLength);
	MathVector before(0,1,0); MathVector current;
	vector<Handle(Geom_BSplineCurve)>::iterator it, et;
	it = segments.begin(); et = segments.end();
	while(it != et){
		GeomAdaptor_Curve gac(*it);
		current = MathVector((*it)->StartPoint(), (*it)->EndPoint());
		WAYROT wr;
		wr.angle  = getTangentStart(*it).angleTo(getTangentEnd(*it));
		wr.length = GCPnts_AbscissaPoint::Length(gac);
		wr.dir    = getDirection(before, current);
		res.push_back(wr);
		before = current;
		++it;
	}
#if 1
	// DEBUG //
	vector<WAYROT>::iterator wit, wet;
	wit = res.begin(); wet = res.end();
	while(wit != wet){
		wit->print();
		++wit;
	}
	///////////
#endif

	return res;
}
