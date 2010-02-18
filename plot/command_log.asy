import graph;

size (12cm, 12cm);
//string data = "../travel.log";
string data = "../command.log";

file in = input (data).line ().csv ();

string[] columnlabel = in;

real[][] A = in.dimension (0, 0);
A = transpose (A);
real[] x = A[0];
real[] y = A[1];

draw (graph (x, y));

scale (true);

xaxis ("x", Bottom, LeftTicks);
xaxis (Top);
yaxis ("y", Left, RightTicks (trailingzero));
yaxis (Right);
