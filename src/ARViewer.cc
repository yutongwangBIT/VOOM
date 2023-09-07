/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ARViewer.h"
#include <pangolin/pangolin.h>
// #include <pangolin/gl/opengl_render_state.h>
#include <pangolin/gl/gl.h>
#include "shader.h"
#include "rendertree.h"
#include "ColorManager.h"
#include "Ellipsoid.h"

#include <mutex>
#include <future>
#include <unistd.h>
#include <random>


namespace ORB_SLAM2
{

ARViewer::ARViewer(System* pSystem, FrameDrawer *pFrameDrawer, Map *pMap, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMap(pMap), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mbPaused(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    w_ = fSettings["Camera.width"];
    h_ = fSettings["Camera.height"];
    if(w_ < 1 || h_ < 1)
    {
        w_ = 640;
        h_ = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    fx_ = fSettings["Camera.fx"];
    fy_ = fSettings["Camera.fy"];
    cx_ = fSettings["Camera.cx"];
    cy_ = fSettings["Camera.cy"];

    K_ << 2 * fx_ / w_,  0,   (w_ - 2 * cx_) / w_, 0, 
          0, 2 * fy_ / h_, (-h_ + 2 * cy_) / h_, 0,
          0, 0, (-zfar_ - znear_)/(zfar_ - znear_), -2 * zfar_ * znear_ / (zfar_ - znear_),
          0, 0, -1, 0;
}

void ARViewer::UpdateFrame(cv::Mat img)
{
    img.copyTo(frame_);
}

void RenderToViewportFlipY(pangolin::GlTexture& tex)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float d = 0.99999;

    GLfloat sq_vert[] = { -1,-1, d, 1,-1, d,  1, 1, d,  -1, 1, d };
    glVertexPointer(3, GL_FLOAT, 0, sq_vert);
    glEnableClientState(GL_VERTEX_ARRAY);

    GLfloat sq_tex[]  = { 0,1,  1,1,  1,0,  0,0  };
    glTexCoordPointer(2, GL_FLOAT, 0, sq_tex);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glEnable(GL_TEXTURE_2D);
    tex.Bind();

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glDisable(GL_TEXTURE_2D);
}


void ARViewer::DrawMapObjects()
{
    const std::vector<MapObject*> objects = mpMap->GetAllMapObjects();

    glPointSize(1);
    const auto& color_manager = CategoryColorsManager::GetInstance();
    glLineWidth(2);
    for (auto *obj : objects) {
        cv::Scalar c;
        c = obj->GetTrack()->GetColor();
        glColor3f(static_cast<double>(c(2)) / 255,
                  static_cast<double>(c(1)) / 255,
                  static_cast<double>(c(0)) / 255);
        const Ellipsoid& ell = obj->GetEllipsoid();
        auto pts = ell.GeneratePointCloud();
        int i = 0;
        while (i < pts.rows()) {
            glBegin(GL_LINE_STRIP);
            for (int k = 0; k < 50; ++k, ++i){
                glVertex3f(pts(i, 0), pts(i, 1), pts(i, 2));
            }
            glEnd();
        }
    }


}





void ARViewer::DrawLines() {
    glLineWidth(1);
    for (const auto& l : lines_) {
        const auto& a = l.first;
        const auto& b = l.second;
        pangolin::glDrawLine(a[0], a[1], a[2], b[0], b[1], b[2]);
    }
}


void ARViewer::Run()
{

    std::cout << "Run AR Viewer" << std::endl;
    mbFinished = false;
    mbStopped = false;

    double w = w_;
    double h = h_;
    pangolin::CreateWindowAndBind("ORB-SLAM2: AR Viewer", w, h);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlMatrix K(K_);
    pangolin::OpenGlMatrix Rt(Rt_);
    
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(K, Rt);

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -w / h)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::GlTexture bg_tex_(w_, h_, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);


     enum class RenderMode { uv=0, tex, color, normal, matcap, vertex, num_modes };
    const std::string mode_names[] = {"SHOW_UV", "SHOW_TEXTURE", "SHOW_COLOR", "SHOW_NORMAL", "SHOW_MATCAP"};
    RenderMode current_mode = RenderMode::color;
    pangolin::AxisDirection spin_direction = pangolin::AxisNone;
    std::vector<std::future<pangolin::Geometry>> geom_to_load;
    std::vector<std::string>  fn = {"../Data/ball.ply"};
     for(const auto& filename : fn)
    {
        geom_to_load.emplace_back(std::async(std::launch::async,[filename](){
            return pangolin::LoadGeometry(filename);
        }) );
    }
    size_t matcap_index = 0;
    std::vector<std::string>  mn;
    mn.push_back("");
    std::vector<pangolin::GlTexture> matcaps = TryLoad<pangolin::GlTexture>(mn, [](const std::string& f){
        return pangolin::GlTexture(pangolin::LoadImage(f));
    });


    RenderNode root;
    std::vector<std::shared_ptr<GlGeomRenderable>> renderables;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 0) = 0.2;
    T(1, 1) = 0.2;
    T(2, 2) = 0.2;
    T(2, 3) = 2;
    auto spin_transform = std::make_shared<FixedTransform>(T);
    
    

    // Pull one piece of loaded geometry onto the GPU if ready
    auto LoadGeometryToGpu = [&]()
    {
        for(auto& future_geom : geom_to_load) {
            if( future_geom.valid() && is_ready(future_geom) ) {
                auto geom = future_geom.get();
                auto aabb = pangolin::GetAxisAlignedBox(geom);
                renderables.push_back(std::make_shared<GlGeomRenderable>(pangolin::ToGlGeometry(geom), aabb));
            }
        }
    };




    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&](const RenderMode mode){
        current_mode = mode;
        default_prog.ClearShaders();
        std::map<std::string,std::string> prog_defines;
        for(int i=0; i < (int)RenderMode::num_modes-1; ++i) {
            prog_defines[mode_names[i]] = std::to_string((int)mode == i);
        }
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::default_model_shader, prog_defines);
        default_prog.Link();
    };

    LoadProgram(current_mode);





    Eigen::Matrix4d gl_to_cv = Eigen::Matrix4d::Identity();
    gl_to_cv << 1, 0.0, 0.0, 0.0, 
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, -1.0, 0.0, 
                0.0, 0.0, 0.0, 1.0;
    int iter = 0;
    while(!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        if (frame_.cols == w_ && frame_.rows == h_) {

            bg_tex_.Upload(frame_.data, GL_BGR, GL_UNSIGNED_BYTE);
            LoadGeometryToGpu();

            const std::vector<MapObject*> objects = mpMap->GetAllMapObjects();
            root.edges.clear();
            for (auto *obj : objects) {

                Eigen::Vector3d c =  obj->GetEllipsoid().GetCenter();
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

                if (!fix_mesh_size_ || size_by_object_.find(obj) == size_by_object_.end()) {
                    Eigen::Vector3d axes =  obj->GetEllipsoid().GetAxes();
                    size_by_object_[obj] = axes.maxCoeff();
                }
                double max_size = size_by_object_[obj];

                T(0, 0) = max_size;
                T(1, 1) = max_size;
                T(2, 2) = max_size;
                T.block<3, 1>(0, 3) = c.cast<float>();
                auto transform = std::make_shared<FixedTransform>(T);
                RenderNode::Edge edge = { transform, { renderables[0] , {} } };
                root.edges.emplace_back(std::move(edge));
            }

            pangolin::OpenGlMatrix Rt(Rt_);
            s_cam.SetModelViewMatrix(gl_to_cv * Rt);

            if(d_cam.IsShown()) {
                d_cam.Activate();
                s_cam.Apply();


                if (disp_mesh_) {
                    default_prog.Bind();
                    render_tree(
                            default_prog, root, s_cam.GetProjectionMatrix(), s_cam.GetModelViewMatrix(),
                            nullptr
                        );
                    default_prog.Unbind();
                } else {
                    DrawMapObjects();
                }

                glColor3f(1.0, 1.0, 1.0);
                RenderToViewportFlipY(bg_tex_);

                glDisable(GL_CULL_FACE);
            }

        }
        pangolin::FinishFrame();

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
        ++iter;
    }

    SetFinish();
}

void ARViewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ARViewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ARViewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool ARViewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void ARViewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool ARViewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool ARViewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void ARViewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

bool ARViewer::isPaused()
{
    return mbPaused;
}

}