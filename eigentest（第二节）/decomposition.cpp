#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 100

int main(int argc, char** argv)
{
    Eigen::MatrixXd matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix< double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random( MATRIX_SIZE, 1);

    //QR-decomposition
    Eigen::Matrix< double, MATRIX_SIZE, 1> x=matrix_NN.colPivHouseholderQr().solve(v_Nd);

    //cholesky-decomposition
    Eigen::MatrixXd matrix_NN2 = matrix_NN.transpose()*matrix_NN;
    x=matrix_NN2.llt().solve(v_Nd);
    cout<<x<<endl;

//    Eigen::Matrix4d A();
//    A << 2,-1,-1,1,
//            1,1,-2,1,
//            4,-6,2,-2,
//            3,6,-9,7;
//
//    Eigen::Vector4d B;
//    B << 2,4,4,9;
//
//    Eigen::Matrix< double, 4, 1> z=A.colPivHouseholderQr().solve(B);
//    cout<<z<<endl;
//
//    Eigen::Matrix2f C, D;
//    C << 2, -1, -1, 3;
//    D << 1, 2, 3, 1;
//    //用llt，C必须是正定阵，ldlt必须是半正定阵,随机阵自己转置再乘自己就正定了
//    Eigen::Matrix< float, 2 ,2> w=C.llt().solve(D);
//
//    cout<<w<<endl;


    return 0;



}