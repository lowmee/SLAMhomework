#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string groundtruth_file = "../groundtruth.txt";
string estimated_file = "../estimated.txt";

ifstream if_GT;
ifstream if_Es;

Sophus::SE3 T_W_g;
Sophus::SE3 T_W_e;

Eigen::Vector3d t_g;
Eigen::Vector3d t_e;

Eigen::Quaterniond q_g;
Eigen::Quaterniond q_e;

double time_g;
double time_e;

Eigen::Matrix<double,6,1> kesi;
double err=0;
double RMSE=0;

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e;


    /// implement pose reading code
    // start your code here (5~10 lines)

    if_GT.open(groundtruth_file.c_str());
    if_Es.open(estimated_file.c_str());

    if(!if_Es.is_open() || !if_GT.is_open()){   //.is_open()
        cout<<"file is empty"<<endl;
        return -1;
    }

    int num = 0;
    string sGTline,sEsline;
    while(getline(if_GT,sGTline) && getline(if_Es, sEsline) && !sEsline.empty() && !sGTline.empty()) {
        istringstream issGT(sGTline);
        istringstream issEs(sEsline);

        issGT >> time_g >>t_g[0]>>t_g[1]>>t_g[2]>>q_g.x()>>q_g.y()>>q_g.z()>>q_g.w();
        issEs >> time_e >>t_e[0]>>t_e[1]>>t_e[2]>>q_e.x()>>q_e.y()>>q_e.z()>>q_e.w();

        T_W_g = Sophus::SE3(q_g, t_g);
        T_W_e = Sophus::SE3(q_e, t_e);

        kesi = (T_W_g.inverse() * T_W_e).log();
        err += kesi.transpose() * kesi;
        num++;

        poses_g.push_back(T_W_g);
        poses_e.push_back(T_W_e);

    }
    if_GT.close();
    if_Es.close();

    RMSE = sqrt(err/num);
    cout << "RMSE :" <<RMSE<< endl;


    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses_g,poses_e);


    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty() && poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            //glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());  渐变色
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }


        for (size_t i = 0; i < poses2.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p3 = poses2[i], p4 = poses2[i + 1];
            glVertex3d(p3.translation()[0], p3.translation()[1], p3.translation()[2]);
            glVertex3d(p4.translation()[0], p4.translation()[1], p4.translation()[2]);
            glEnd();
        }


        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }


}
