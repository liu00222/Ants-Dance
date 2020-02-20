#ifndef GFX_TEST_H_
#define GFX_TEST_H_

#include <mingfx.h>
using namespace mingfx;

#include <string>
#include <vector>

class GfxTest : public GraphicsApp {
public:
  
    GfxTest();
    virtual ~GfxTest();
    
    void OnPauseBtnPressed();
    
    void UpdateSimulation(double dt);
    
    void InitOpenGL();
    void InitNanoGUI();
    
    void DrawUsingOpenGL();
      
private:    
    
    // Sets up the computer graphics camera
    Matrix4 rot_matrix_;
    Matrix4 view_matrix_;
    Matrix4 proj_matrix_;
    
    Mesh teapot_;
    DefaultShader shader_;
    
    // Gui elements
    nanogui::Button  *pauseBtn_;
    bool playing_;
};

#endif