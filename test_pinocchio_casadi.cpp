// test_pinocchio_casadi.cpp
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/multibody/model.hpp>

int main()
{
    pinocchio::ModelTpl<casadi::SX> cmodel;
    std::cout << "Pinocchio + CasADi works\n";

    std::cout << "------- SX -------" << "\n";
    casadi::SX x = casadi::SX::sym("x");
    std::cout << "x: " << x << "\n";

    casadi::SX Y = casadi::SX::sym("y", 4, 2);
    std::cout << "Y: " << Y << "\n";

    casadi::SX f = x*3;
    f = f + Y;
    std::cout << "f: " << f << "\n" << f.size() << "\n";

    casadi::SX B1 = casadi::SX::zeros(4,5);
    std::cout << "B1: " << B1 << "\n";

    casadi::SX B2 = casadi::SX(4,5);
    std::cout << "B2: " << B2 << "\n";

    casadi::SX B4 = casadi::SX::eye(4);
    std::cout << "B4: " << B4 << "\n";

    std::cout << "------- MX -------" << "\n";
    casadi::MX x1 = casadi::MX::sym("x", 2, 2);
    std::cout << "x: " << x1 << "\n";

    casadi::MX y1 = casadi::MX::sym("y");
    std::cout << "y: " << y1 << "\n";

    casadi::MX f1 = 3*x1 + y1;
    std::cout << "f: " << f1 << "\n" << f.size() << "\n";

    std::cout << "x[0,0]: " << x1(0, 0) << "\n";

    std::cout << "------- Arithmetic -------\n";
    casadi::SX x2 = casadi::SX::sym("x");
    casadi::SX y2 = casadi::SX::sym("y", 2, 2);
    
    std::cout << sin(y2) - x2 << "\n";
    std::cout << "Mult element-wise: " << y2*y2 << "\n";
    std::cout << "Matrix mult: " << mtimes(y2, y2) << "\n";

    std::cout << "------- Algebra -------\n";
    casadi::MX b = casadi::MX::sym("b", 3);
    casadi::MX A = casadi::MX::sym("A", 3, 3);
    std::cout << solve(A, b) << "\n";

    std::cout << "------- Function Objects -------\n";
    casadi::SX x3 = casadi::SX::sym("x", 2);
    casadi::SX y3 = casadi::SX::sym("y");
    casadi::Function f3 = casadi::Function("f", {x3,y3}, {x3, sin(y3)*x3});
    std::cout << "f: " << f3 << "\n";

    return 0;
}