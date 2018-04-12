#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int agrc, char** agrv)
{
    Eigen::Quaterniond q1(0.55,0.3,0.2,0.2);
    Eigen::Vector3d t1(0.7,1.1,0.2);

    Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);
    Eigen::Vector3d t2(-0.1,0.4,0.8);

    q1.normalize();
    q2.normalize();

    Eigen::Isometry3d Tw1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tw2 = Eigen::Isometry3d::Identity();

    Tw1.rotate(q1);
    Tw1.pretranslate(t1);
    Tw2.rotate(q2);
    Tw2.pretranslate(t2);

    Eigen::Vector3d p1(0.5,-0.1,0.2);
    Eigen::Vector3d p2;
    Eigen::Vector3d pw;

    pw = Tw1.inverse()*p1;
    p2 = Tw2*pw;

    cout<<p2.transpose()<<endl;

    return 0;

}