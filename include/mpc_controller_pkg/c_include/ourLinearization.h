#ifndef OURLINEARIZATION_H_
#define OURLINEARIZATION_H_
void updateA(double x[9], double u[8], double (*M)[9] );
void updateB(double x[9], double u[8], double (*M)[8] );
void updateSystem(double x[9], double u[8], double (*A)[9], double (*B)[8]);
#endif
