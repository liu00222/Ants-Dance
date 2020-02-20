#include "gfx_test.h"

#include <iostream>
#include <sstream>

GfxTest::GfxTest() : GraphicsApp(1024,768, "GfxTest"), playing_(true) {
}
    
    
GfxTest::~GfxTest() {
}

    
void GfxTest::InitNanoGUI() {
    // Setup the GUI window
    nanogui::Window *window = new nanogui::Window(screen(), "Controls");
    window->setPosition(Eigen::Vector2i(10, 10));
    window->setSize(Eigen::Vector2i(400,200));
    window->setLayout(new nanogui::GroupLayout());
    
    pauseBtn_ = new nanogui::Button(window, "Pause");
    pauseBtn_->setCallback(std::bind(&GfxTest::OnPauseBtnPressed, this));
    pauseBtn_->setTooltip("Toggle playback.");
        
    screen()->performLayout();
}



void GfxTest::OnPauseBtnPressed() {
    playing_ = !playing_;
    if (playing_) {
        pauseBtn_->setCaption("Pause");
    }
    else {
        pauseBtn_->setCaption("Play");
    }
}


void GfxTest::UpdateSimulation(double dt)  {
    static const float radians_per_sec = GfxMath::ToRadians(25.0);
    if (playing_) {
        rot_matrix_ = rot_matrix_ * Matrix4::RotationY(dt * radians_per_sec);
    }
}


void GfxTest::InitOpenGL() {
    // Set up the camera in a good position to see the entire field
    proj_matrix_ = Matrix4::Perspective(60, aspect_ratio(), 0.01, 100);
    view_matrix_ = Matrix4::LookAt(Point3(0,0,4), Point3(0,0,0), Vector3(0,1,0));
    glClearColor(0.0, 0.0, 0.0, 1);
    
    teapot_.LoadFromOBJ(Platform::FindMinGfxDataFile("teapot.obj"));
    shader_.Init();
}


void GfxTest::DrawUsingOpenGL() {
    for (int r=-2; r<3; r++) {
        for (int c=-2; c<3; c++) {
            Matrix4 model_matrix = Matrix4::Translation(Vector3(r,c,0)) * rot_matrix_;
            shader_.Draw(model_matrix, view_matrix_, proj_matrix_, &teapot_, DefaultShader::MaterialProperties());
        }
    }
}





