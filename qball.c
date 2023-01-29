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

static Point2 ctlcen;		/* center of qball */
static double ctlrad;		/* radius of qball */
static Quaternion *axis;	/* constraint plane orientation, 0 if none */

static double
fmin(double a, double b)
{
	return a < b? a: b;
}

/*
 * Convert a mouse point into a unit quaternion, flattening if
 * constrained to a particular plane.
 */
static Quaternion
mouseq(Point2 p)
{
	double qx = (p.x-ctlcen.x)/ctlrad;
	double qy = (p.y-ctlcen.y)/ctlrad;
	double rsq = qx*qx + qy*qy;
	double l;
	Quaternion q;

	if(rsq > 1){
		rsq = sqrt(rsq);
		q.r = 0;
		q.i = qx/rsq;
		q.j = qy/rsq;
		q.k = 0;
	}else{
		q.r = 0;
		q.i = qx;
		q.j = qy;
		q.k = sqrt(1-rsq);
	}

	if(axis != nil){
		l = q.i*axis->i + q.j*axis->j + q.k*axis->k;
		q.i -= l*axis->i;
		q.j -= l*axis->j;
		q.k -= l*axis->k;
		l = sqrt(q.i*q.i + q.j*q.j + q.k*q.k);
		if(l != 0){
			q.i /= l;
			q.j /= l;
			q.k /= l;
		}
	}
	return q;
}

void
qball(Rectangle r, Point mxy, Quaternion *orient, Quaternion *ap)
{
	Quaternion down;
	Point2 rmin, rmax;
	Point2 rad;

	if(orient == nil)
		return;

	axis = ap;
	rmin = Pt2(r.min.x, r.min.y);
	rmax = Pt2(r.max.x, r.max.y);
	ctlcen = divpt2(addpt2(rmin, rmax), 2);
	rad = divpt2(subpt2(rmax, rmin), 2);
	ctlrad = fmin(rad.x, rad.y);
	down = invq(mouseq(Pt2(mxy.x, mxy.y)));
	*orient = mulq(*orient, mulq(down, mouseq(Pt2(mxy.x, mxy.y))));
}
