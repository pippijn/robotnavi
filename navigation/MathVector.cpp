#include "MathVector.h"

MathVector::MathVector(void){}

MathVector::MathVector(double _x, double _y, double _z){
	x = _x;
	y = _y;
	z = _z;
}

MathVector::MathVector(Point from, Point to){
	x = to.x - from.x;
	y = to.y - from.y;
	z = to.z - from.z;
}

MathVector::~MathVector(void){}

MathVector MathVector::copy() const{
	return MathVector(x, y, z);
}

MathVector MathVector::multiply(MathVector const& other) const{
	return MathVector(x*other.x, y*other.y, z*other.z);
}

MathVector MathVector::add(MathVector const& other) const{
	return MathVector(x+other.x, y+other.y, z+other.z);
}

MathVector MathVector::sub(MathVector const& other) const{
	return MathVector(x-other.x, y-other.y, z-other.z);
}

MathVector MathVector::cross(MathVector const& other) const{
	return MathVector(
		((y*other.z)-(z*other.y)),
		((z*other.x)-(x*other.z)),
		((x*other.y)-(y*other.x))
	);
}

double MathVector::dotProduct(MathVector const& other) const{
	return ((x*other.x)+(y*other.y)+(z*other.z));
}

double MathVector::norm() const{
	return sqrt( ((x*x)+(y*y)+(z*z)) );
}

void MathVector::tol(gp_Pnt &pnt)const{
	pnt.SetX(tol(pnt.X()));
	pnt.SetY(tol(pnt.Y()));
	pnt.SetZ(tol(pnt.Z()));
}

double MathVector::tol(double value)const{
	double tmp = value;
	if(tmp < 0)
		tmp *= -1;
	if(tmp <= 1e-5)
		return 0;
	return value;
}

MathVector MathVector::getOrthogonalVector()const{
	gp_Ax1 rotationAxe(gp_Pnt(0,0,0), gp_Dir(0,0,1));
	gp_Trsf transformation;
	transformation.SetRotation(rotationAxe, gradToRadian(90));
	BRepBuilderAPI_GTransform transformer(BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,0,0), gp_Pnt(x,y,z))), transformation);
	TopoDS_Shape rotatedShape = transformer.Shape();
	TopoDS_Edge edg = TopoDS::Edge(rotatedShape);

	TopExp_Explorer exp(edg, TopAbs_VERTEX);
	bool first = true;
	TopoDS_Vertex fv, lv;
	while(exp.More()){
		if(first){
			fv = TopoDS::Vertex(exp.Current());
			first = false;
		}else{
			lv =  TopoDS::Vertex(exp.Current());
		}
		exp.Next();
	}

	gp_Pnt from = BRep_Tool::Pnt(fv);
	gp_Pnt to = BRep_Tool::Pnt(lv);
	tol(from); tol(to);

	return MathVector(Point(from), Point(to));
}

void MathVector::operator=(const MathVector vec){
	x = vec.x;
	y = vec.y;
	z = vec.z;
}

bool MathVector::operator==(const MathVector other){
	if(x == other.x)
		if(y == other.y)
			if(z == other.z)
				return true;
	return false;
}

bool MathVector::operator!=(const MathVector other){
	if(x == other.x)
		if(y == other.y)
			if(z == other.z)
				return false;
	return true;
}

double MathVector::angleTo(MathVector const& other) const{
	double dividend = dotProduct(other);
	if(dividend < 0)
		dividend *= (-1.0);
	double divisor = (norm() * other.norm());
	return (acos ((dividend / divisor)) * 180.0 / PI); // In Grad
}

MathVector MathVector::unity () const {
  return MathVector (x / norm (), y / norm (), z / norm ());
}

TopoDS_Edge MathVector::getTopoDS_Edge(){
	return BRepBuilderAPI_MakeEdge(GC_MakeSegment(gp_Pnt(0,0,0), gp_Pnt(x,y,z)));
}

double MathVector::gradToRadian(double grad) const{
	return (grad * (PI / 180));
}

gp_Pnt MathVector::getGP_Pnt()const{
  return gp_Pnt(x,y,z);
}
