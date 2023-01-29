int objread(char *, Triangle3 **);
Triangle Trian(int, int, int, int, int, int);
Triangle Trianpt(Point, Point, Point);
Point centroid(Triangle);
void triangle(Image *, Triangle, int, Image *, Point);
void filltriangle(Image *, Triangle, Image *, Point);
Triangle rotatriangle(Triangle, double, Point);
Vector Vec(double, double);
Vector addvec(Vector, Vector);
Vector subvec(Vector, Vector);
Vector mulvec(Vector, double);
double dotvec(Vector, Vector);
Vector normvec(Vector);
Vector3 Vec3(double, double, double);
Vector3 Vecquat(Quaternion);
Vector3 addvec3(Vector3, Vector3);
Vector3 subvec3(Vector3, Vector3);
Vector3 mulvec3(Vector3, double);
double dotvec3(Vector3, Vector3);
Vector3 crossvec(Vector3, Vector3);
Vector3 normvec3(Vector3);
void addm(Matrix, Matrix);
void subm(Matrix, Matrix);
void mulm(Matrix, Matrix);
void transm(Matrix);
double detm(Matrix);
Vector3 mulvecm(Vector3, Matrix);
Quaternion Quat(double, double, double, double);
Quaternion Quatvec(double, Vector3);
Quaternion addq(Quaternion, Quaternion);
Quaternion subq(Quaternion, Quaternion);
Quaternion mulq(Quaternion, Quaternion);
Quaternion smulq(Quaternion, double);
Quaternion sdivq(Quaternion, double);
double dotq(Quaternion, Quaternion);
Quaternion invq(Quaternion);
double qlen(Quaternion);
Quaternion normq(Quaternion);
Vector3 qrotate(Vector3, Vector3, double);
void qball(Rectangle, Point, Quaternion*, Quaternion*);
void qb(Rectangle, Point, Point, Quaternion*, Quaternion*);
double round(double);
Point rotatept(Point, double, Point);
double hypot3(double, double, double);
void *emalloc(ulong);
void *erealloc(void *, ulong);
