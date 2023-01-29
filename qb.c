/*
 * Ken Shoemake's Quaternion rotation controller
 */
#include <u.h>
#include <libc.h>
#include <draw.h>
#include "dat.h"
#include "fns.h"

typedef struct Point2 Point2;
struct Point2 {
	double x, y;
};

static Point2
Pt2(double x, double y)
{
	return (Point2){x, y};
}

static Point2
addpt2(Point2 a, Point2 b)
{
	return (Point2){a.x+b.x, a.y+b.y};
}

static Point2
subpt2(Point2 a, Point2 b)
{
	return (Point2){a.x-b.x, a.y-b.y};
}

static Point2
divpt2(Point2 p, double s)
{
	return (Point2){p.x/s, p.y/s};
}

/*
 * Convert a mouse point into a unit quaternion, flattening if
 * constrained to a particular plane.
 */
static Quaternion
mouseq(Point2 p, Quaternion *axis){
	double l;
	Quaternion q;
	double rsq = p.x*p.x + p.y*p.y;

	if(rsq > 1){
		rsq = sqrt(rsq);
		q.k = 0;
		q.i = p.x/rsq;
		q.j = p.y/rsq;
		q.r = 0;
	}
	else{
		q.r = 0;
		q.i = p.x;
		q.j = p.y;
		q.k = sqrt(1 - rsq);
	}
	if(axis != nil){
		l    = q.i*axis->i + q.j*axis->j + q.k*axis->k;
		q.i -= l*axis->i;
		q.j -= l*axis->j;
		q.k -= l*axis->k;
		l    = sqrt(q.i*q.i + q.j*q.j + q.k*q.k);
		if(l != 0.){
			q.i /= l;
			q.j /= l;
			q.k /= l;
		}
	}
	return q;
}

void
qb(Rectangle r, Point p1, Point p2, Quaternion *orient, Quaternion *axis){
	Quaternion q, down;
	Point2 rmin, rmax;
	Point2 ctlcen, ctlrad;
	double qx, qy;

	rmin = Pt2(r.min.x, r.min.y);
	rmax = Pt2(r.max.x, r.max.y);
	ctlcen = divpt2(addpt2(rmin, rmax), 2);
	ctlrad = divpt2(subpt2(rmax, rmin), 2);
	qx  = (p1.x-ctlcen.x)/ctlrad.x;
	qy  = (ctlcen.y-p1.y)/ctlrad.y;
	down = invq(mouseq(Pt2(qx, qy), axis));

	q = *orient;
	qx  = (p2.x-ctlcen.x)/ctlrad.x;
	qy  = (ctlcen.y-p2.y)/ctlrad.y;
	*orient = mulq(q, mulq(down, mouseq(Pt2(qx, qy), axis)));
}
