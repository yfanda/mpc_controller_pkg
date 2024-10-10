#ifndef TRACKING_H
#define TRACKING_H
#include "grampc.h"

void Update_des(typeGRAMPC *grampc,int line_number);
void Update_Terminial_Constraints(typeGRAMPC *grampc, int line_number, typeRNum factor);
void desReader(double* xdes, int num_cols, const char* filename, int line_number, double Thor, double dt);
void lineReader(double* xdes, int num_cols, const char* filename, int line_number);
#endif