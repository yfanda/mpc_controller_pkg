#include "tracking.h"
#include "grampc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

//#pragma warning(disable:4996)

/* void Update_des(typeGRAMPC *grampc, int line_number){
    int nx = grampc->param->Nx;
    int nu = grampc->param->Nu;
    typeRNum Thor = grampc->param->Thor;
    typeRNum dt = grampc->param->dt;
    int horizon = (Thor/dt)+1;
    int x_horizon_size = nx * horizon; 
    int u_horizon_size = nu * horizon; 

    typeRNum xdes[x_horizon_size];
    typeRNum udes[u_horizon_size]; 
    typeRNum* xdes;
    typeRNum* udes;
    char* filename1 = "res/x_trag_no_failure_sim.txt";
    desReader(xdes, nx, filename1, line_number, Thor, dt);

    char* filename2 = "res/u_traj_no_failure_sim.txt";
    desReader(udes, nu, filename2, line_number, Thor, dt);
    
    grampc_setparam_real_vector(grampc, "udes", udes);
    grampc_setparam_real_vector(grampc, "xdes", xdes); 
}*/

 void Update_Terminial_Constraints(typeGRAMPC *grampc, int line_number, typeRNum factor){

/*     typeRNum x_predict[9] = { 0 };
    char* filename3 = "res/Traj/x_traj_rotor1_failure_3.txt";
    int line_number_predict = line_number + grampc->param->Thor/ grampc->param->dt;
    lineReader(x_predict, 9, filename3, line_number_predict);

    typeRNum pCons_bu[9] = { 0 };
    typeRNum pCons_bl[9] = { 0 };

    
	
    pCons_bu[0] = sqrt(pow(x_predict[0], 2) + pow(x_predict[1], 2) + pow(x_predict[2], 2)) + 10*factor;
    pCons_bu[1] = atan(x_predict[2] / x_predict[0]) + factor;
    pCons_bu[2] = asin(x_predict[1] / sqrt(pow(x_predict[0], 2) + pow(x_predict[1], 2) + pow(x_predict[2], 2))) + factor;
    for (int h = 3; h < 9; h++) 
    {
        pCons_bu[h] = x_predict[h] + factor;
    }
    MatCopy(grampc->userparam->bu, pCons_bu, 1, 9);

    pCons_bl[0] = sqrt(pow(x_predict[0], 2) + pow(x_predict[1], 2) + pow(x_predict[2], 2)) - 10*factor;
    pCons_bl[1] = atan(x_predict[2] / x_predict[0]) - factor;
    pCons_bl[2] = asin(x_predict[1] / sqrt(pow(x_predict[0], 2) + pow(x_predict[1], 2) + pow(x_predict[2], 2))) - factor;
    for (int h = 3; h < 9; h++)
    {
        pCons_bl[h] = x_predict[h] - factor;
    }

    MatCopy(grampc->userparam->bl,pCons_bl , 1, 9);
 */

} 

void desReader(double* xdes, int NX, const char*filename, int line_number, double Thor, double dt)
{
    int horizon = Thor/dt+1;
    int n = dt/0.01;
    for(int m = 0; m < horizon;m++){
        lineReader(xdes + m*NX, NX, filename, line_number + m*n);

    }
}

void lineReader(double* xdes, int num_cols, const char* filename, int line_number)
{
    char line[109];
    int line_count = 1;
    FILE* file;
    file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Failed to open file: %s\n", filename);
        exit(1);
    }

    while (fgets(line, 109, file) != NULL) {

        if (strlen(line) <= 1) {
            continue;
        }

        if (line_count == line_number) {
            if (strlen(line) > 1) {
                int num_read = 0;
                char* pos = line;
                while (isspace(*pos)) {
                    pos++;
                }
                while (sscanf(pos, "%lf", &xdes[num_read]) == 1 && num_read < num_cols) {
                    num_read++;
                    while (!isspace(*pos) && *pos != '\0') {
                        pos++;
                    }
                    while (isspace(*pos)) {
                        pos++;
                    }
                }
                break; 
            }
        }
        line_count++;
    }

    fclose(file);
}