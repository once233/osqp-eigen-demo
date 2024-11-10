/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2018
 */
#include "math.h"
int main()
{
    // set the preview window
    int mpcWindow = 20;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 12, 12> a;
    Eigen::Matrix<double, 12, 4> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 12, 1> xMax;
    Eigen::Matrix<double, 12, 1> xMin;
    Eigen::Matrix<double, 4, 1> uMax;
    Eigen::Matrix<double, 4, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 12> Q;
    Eigen::DiagonalMatrix<double, 4> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 12, 1> x0;
    Eigen::Matrix<double, 12, 1> xRef;

    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // set the initial and the desired states
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    xRef << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // set MPC problem quantities
    setXDistVMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(12 * (mpcWindow + 1) + 4 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 12 * (mpcWindow + 1) + 4 * mpcWindow);
    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;

    // instantiate the solver
    if (!solver.initSolver())
        return 1;

    // controller input and QPSolution vector
    Eigen::Vector4d ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    int numberOfSteps = 50;

    for (int i = 0; i < numberOfSteps; i++)
    {

        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(12 * (mpcWindow + 1), 0, 4, 1);

        // save data into file
        auto x0Data = x0.data();

        // propagate the model
        x0 = a * x0 + b * ctr;

        // update the constraint bound
        updateConstraintVectors(x0, lowerBound, upperBound);
        if (!solver.updateBounds(lowerBound, upperBound))
            return 1;
    }
    return 0;
}
