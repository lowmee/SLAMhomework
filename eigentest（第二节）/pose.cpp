#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int agrc, char** agrv)
{
    Eigen::Quaterniond q1(0.55,0.3,0.2,0.2);
    Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);

    Eigen::Vector3d t1(0.7,1.1,0.2);
    Eigen::Vector3d t2(-0.1,0.4,0.8);

    q1.normalize();
    q2.normalize();

    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.rotate(q1);
    T1.pretranslate(t1);

    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T1.rotate(q2);
    T1.pretranslate( t2);

    Eigen::Vector3d pw;
    Eigen::Vector3d p1(0.5,-0.1,0.2);
    Eigen::Vector3d p2;

    pw = T1.inverse()*p1;
    p2 = T2*pw;

    cout<<"Q1=\n"<<q1.coeffs()<<endl;
    cout<<"Q2=\n"<<q2.coeffs()<<endl;
    cout<<"T1=\n"<<T1.matrix()<<endl;
    cout<<"T2=\n"<<T2.matrix()<<endl;
    cout<<p2.transpose()<<endl;

    return 0;

}
