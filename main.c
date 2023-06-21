#include <u.h>
#include <libc.h>
#include <thread.h>
#include <draw.h>
#include <mouse.h>
#include <keyboard.h>
#include "dat.h"
#include "fns.h"

Mousectl *mctl;
Keyboardctl *kctl;
Mouse om;
Point orig;
Vector basis;
double Znear, Zfar, Zoff, a;
Matrix proj;
Vector3 light;
double θ, ω;
double t0, Δt;
int dowireframe;
RWLock worldlock;
Quaternion orient;

Mesh model;
/*Triangle3 cube[] = {
	0,0,0, 0,1,0, 1,1,0,
	0,0,0, 1,1,0, 1,0,0,

	1,0,0, 1,1,0, 1,1,1,
	1,0,0, 1,1,1, 1,0,1,

	1,0,1, 1,1,1, 0,1,1,
	1,0,1, 0,1,1, 0,0,1,

	0,0,1, 0,1,1, 0,1,0,
	0,0,1, 0,1,0, 0,0,0,

	0,1,0, 0,1,1, 1,1,1,
	0,1,0, 1,1,1, 1,1,0,

	1,0,1, 0,0,1, 0,0,0,
	1,0,1, 0,0,0, 1,0,0,
};*/

void *
emalloc(ulong n)
{
	void *p;

	p = malloc(n);
	if(p == nil)
		sysfatal("malloc: %r");
	memset(p, 0, n);
	setmalloctag(p, getcallerpc(&n));
	return p;
}

void *
erealloc(void *ptr, ulong n)
{
	void *p;

	p = realloc(ptr, n);
	if(p == nil)
		sysfatal("realloc: %r");
	setrealloctag(p, getcallerpc(&ptr));
	return p;
}

Image *
eallocimage(Display *d, Rectangle r, ulong chan, int repl, ulong col)
{
	Image *i;

	i = allocimage(d, r, chan, repl, col);
	if(i == nil)
		sysfatal("allocimage: %r");
	return i;
}

#pragma varargck type "V" Vector3
int
Vfmt(Fmt *f)
{
	Vector3 v;

	v = va_arg(f->args, Vector3);
	return fmtprint(f, "(%g %g %g)", v.x, v.y, v.z);
}

#pragma varargck type "H" Quaternion
int
Hfmt(Fmt *f)
{
	Quaternion q;

	q = va_arg(f->args, Quaternion);
	return fmtprint(f, "(%g %g %g %g)", q.r, q.i, q.j, q.k);
}

#pragma varargck type "T" Triangle3
int
Tfmt(Fmt *f)
{
	Triangle3 t;

	t = va_arg(f->args, Triangle3);
	return fmtprint(f, "%V%V%V", t.p0, t.p1, t.p2);
}

Point
vectopt(Vector v)
{
	return Pt(v.x, v.y);
}

Point
toscreen(Vector p)
{
	return addpt(orig, Pt(p.x*basis.x, p.y*basis.y));
}

int
depthcmp(void *a, void *b)
{
	Triangle3 *ta, *tb;
	double za, zb;

	ta = (Triangle3 *)a;
	tb = (Triangle3 *)b;
	za = (ta->p0.z + ta->p1.z + ta->p2.z)/3;
	zb = (tb->p0.z + tb->p1.z + tb->p2.z)/3;
	return za > zb ? -1 : 1;
}

void
drawinfo(void)
{
	char buf[128];

	snprint(buf, sizeof buf, "%H", orient);
	string(screen, addpt(screen->r.min, Pt(30,30)), display->white, ZP, font, buf);
}

void
redraw(void)
{
	Triangle3 trans, *vistris;
	Vector3 n;
//	Quaternion Xaxis, Yaxis, Zaxis;
//	Quaternion q;
	Matrix T = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, Zoff,
		0, 0, 0, 1,
	}, S = {
		Dx(screen->r)/2, 0, 0, 0,
		0, Dy(screen->r)/2, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	};
	u32int c;
	int nvistri, i;

	vistris = nil;
	nvistri = 0;
//	Xaxis = Quat(cos(θ/4), sin(θ/4), 0, 0);
//	Yaxis = Quat(cos(θ/6), 0, sin(θ/6), 0);
//	Zaxis = Quat(cos(θ/2), 0, 0, sin(θ/2));
//	q = mulq(Zaxis, mulq(Yaxis, Xaxis));
	lockdisplay(display);
	draw(screen, screen->r, display->black, nil, ZP);
	for(i = 0; i < model.ntri; i++){
//		trans.p0 = Vecquat(mulq(mulq(q, Quatvec(0, model.tris[i].p0)), invq(q)));
//		trans.p1 = Vecquat(mulq(mulq(q, Quatvec(0, model.tris[i].p1)), invq(q)));
//		trans.p2 = Vecquat(mulq(mulq(q, Quatvec(0, model.tris[i].p2)), invq(q)));

//		trans.p0 = Vecquat(mulq(mulq(invq(orient), Quatvec(0, model.tris[i].p0)), orient)); 
//		trans.p1 = Vecquat(mulq(mulq(invq(orient), Quatvec(0, model.tris[i].p1)), orient));
//		trans.p2 = Vecquat(mulq(mulq(invq(orient), Quatvec(0, model.tris[i].p2)), orient)); 

		trans.p0 = Vecquat(mulq(mulq(orient, Quatvec(0, model.tris[i].p0)), invq(orient))); 
		trans.p1 = Vecquat(mulq(mulq(orient, Quatvec(0, model.tris[i].p1)), invq(orient)));
		trans.p2 = Vecquat(mulq(mulq(orient, Quatvec(0, model.tris[i].p2)), invq(orient)));

		trans.p0 = mulvecm(trans.p0, T);
		trans.p1 = mulvecm(trans.p1, T);
		trans.p2 = mulvecm(trans.p2, T);

		n = normvec3(crossvec(
			subvec3(trans.p1, trans.p0),
			subvec3(trans.p2, trans.p0)));
		if(dotvec3(n, trans.p0) < 0){
			trans.p0 = mulvecm(trans.p0, proj);
			trans.p1 = mulvecm(trans.p1, proj);
			trans.p2 = mulvecm(trans.p2, proj);
			trans.p0 = addvec3(trans.p0, Vec3(1, 1, 0));
			trans.p1 = addvec3(trans.p1, Vec3(1, 1, 0));
			trans.p2 = addvec3(trans.p2, Vec3(1, 1, 0));
			trans.p0 = mulvecm(trans.p0, S);
			trans.p1 = mulvecm(trans.p1, S);
			trans.p2 = mulvecm(trans.p2, S);
			c = 0xff*fabs(dotvec3(n, light));
			trans.tx = eallocimage(display, Rect(0, 0, 1, 1), screen->chan, 1, c<<24|c<<16|c<<8|0xff);
			vistris = erealloc(vistris, ++nvistri*sizeof(Triangle3));
			vistris[nvistri-1] = trans;
		}
	}
	qsort(vistris, nvistri, sizeof(Triangle3), depthcmp);
	for(i = 0; i < nvistri; i++){
		filltriangle(screen, Trianpt(
			toscreen(Vec(vistris[i].p0.x, vistris[i].p0.y)),
			toscreen(Vec(vistris[i].p1.x, vistris[i].p1.y)),
			toscreen(Vec(vistris[i].p2.x, vistris[i].p2.y))
			), vistris[i].tx, ZP);
		if(dowireframe)
			triangle(screen, Trianpt(
				toscreen(Vec(vistris[i].p0.x, vistris[i].p0.y)),
				toscreen(Vec(vistris[i].p1.x, vistris[i].p1.y)),
				toscreen(Vec(vistris[i].p2.x, vistris[i].p2.y))
				), 0, display->black, ZP);
		freeimage(vistris[i].tx);
	}
	free(vistris);
	drawinfo();
	flushimage(display, 1);
	unlockdisplay(display);
}

void
lmb(void)
{
//	double tmp;

//	qball(screen->r, mctl->xy, &orient, nil);
	if(om.buttons & 1)
		qb(screen->r, om.xy, mctl->xy, &orient, nil);
//	fprint(2, "orient %H\n", orient);
}

void
rmb(void)
{
	enum {
		DOWIREFRM,
	};
	static char *items[] = {
	 [DOWIREFRM]	"toggle wireframe",
		nil,
	};
	static Menu menu = { .item = items };

	switch(menuhit(3, mctl, &menu, nil)){
	case DOWIREFRM:
		dowireframe ^= 1;
		break;
	}
	t0 = nsec();
}

void
mouse(void)
{
	if(mctl->buttons & 1)
		lmb();
	if(mctl->buttons & 4)
		rmb();
	if(mctl->buttons & 8)
		Zoff -= 0.2;
	if(mctl->buttons & 16)
		Zoff += 0.2;
	om = mctl->Mouse;
}

void
key(Rune r)
{
	switch(r){
	case Kdel:
	case 'q':
		threadexitsall(nil);
	case Kpgup:
		Zoff += 0.2;
		break;
	case Kpgdown:
		Zoff -= 0.2;
		break;
	}
}

void
resized(void)
{
	lockdisplay(display);
	if(getwindow(display, Refnone) < 0)
		fprint(2, "can't reattach to window\n");
	unlockdisplay(display);
	orig = Pt(screen->r.min.x, screen->r.max.y);
	a = (double)Dy(screen->r)/Dx(screen->r);
	proj[0][0] = a*(1/tan(FOV/2*DEG));
	redraw();
}

void
scrsynproc(void *)
{
	threadsetname("scrsynproc");
	for(;;){
		rlock(&worldlock);
		redraw();
		runlock(&worldlock);
		sleep(SEC/FPS);
	}
}

void
usage(void)
{
	fprint(2, "usage: %s\n", argv0);
	threadexitsall("usage");
}

void
threadmain(int argc, char *argv[])
{
	Rune r;

	fmtinstall('V', Vfmt);
	fmtinstall('H', Hfmt);
	fmtinstall('T', Tfmt);
	ARGBEGIN{
	default: usage();
	}ARGEND;

	if(initdraw(nil, nil, "threedee") < 0)
		sysfatal("initdraw: %r");
	if((mctl = initmouse(nil, screen)) == nil)
		sysfatal("initmouse: %r");
	if((kctl = initkeyboard(nil)) == nil)
		sysfatal("initkeyboard: %r");
	orig = Pt(screen->r.min.x, screen->r.max.y);
	basis = Vec(1, -1);
	orient = Quat(1, 1, 0, 0);
	Znear = 0.1;
	Zfar = 1000;
	Zoff = 2;
	a = (double)Dy(screen->r)/Dx(screen->r);
	proj[0][0] = a*(1/tan(FOV/2*DEG));
	proj[1][1] = 1/tan(FOV/2*DEG);
	proj[2][2] = Zfar / (Zfar + Znear);
	proj[2][3] = -Zfar * Znear / (Zfar + Znear);
	proj[3][2] = 1;
	light = Vec3(0, 0, -1);
	if((model.ntri = objread("mdl/rocket.obj", &model.tris)) < 0)
		sysfatal("objread: %r");
	display->locking = 1;
	unlockdisplay(display);
	proccreate(scrsynproc, nil, STACK);
	ω = 1;
	t0 = nsec();
	for(;;){
		enum {MOUSE, RESIZE, KBD};
		Alt a[] = {
			{mctl->c, &mctl->Mouse, CHANRCV},
			{mctl->resizec, nil, CHANRCV},
			{kctl->c, &r, CHANRCV},
			{nil, nil, CHANNOBLK}
		};
		wlock(&worldlock);
		switch(alt(a)){
		case MOUSE: mouse(); break;
		case RESIZE: resized(); break;
		case KBD: key(r); break;
		}
		Δt = (nsec()-t0)/1e9;
		θ += ω*Δt;
		t0 += Δt*1e9;
		wunlock(&worldlock);
		sleep(SEC/FPS);
	}
}
