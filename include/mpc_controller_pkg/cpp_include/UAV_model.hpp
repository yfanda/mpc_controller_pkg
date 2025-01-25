#ifndef UAV_MODEL_HPP
#define UAV_MODEL_HPP

#include "problem_description.hpp" 
#include <iostream> 
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <sstream>

class UAVModel : public grampc::ProblemDescription
{
public:

    UAVModel(typeRNum Thor, typeRNum dt);
    ~UAVModel() = default;

    virtual void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT) override;

    virtual void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) override;
    virtual void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p) override;
    virtual void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p) override;

    virtual void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;

    virtual void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;

    virtual void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override;

    virtual void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override;
    virtual void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override;

    int sign(double x);

    std::vector<std::vector<double>> x_traj_cache;
    std::vector<std::vector<double>> u_traj_cache;

    void updateTrajData(int iMPC) ;
    void loadFileToCache(const std::string& filename, int num_cols, std::vector<std::vector<double>>& cache);
    void initializeCache(const std::string& x_traj_file, const std::string& u_traj_file);
    void readFromCache(double* dest, const std::vector<std::vector<double>>& cache, int line_number, int num_rows, int num_cols);
    std::array<double, 9> rk4(const std::array<double, 9>& state, const std::array<double, 8>& input);
    void getInitialValues(typeRNum* x0, int x_size, typeRNum* u0, int u_size);

public:
    std::vector<double> userparam;  
    typeRNum Thor;
    typeRNum dt;
};

#endif 