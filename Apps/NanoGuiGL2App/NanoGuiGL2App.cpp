
//========================================================================
// Simple GLFW example
// Copyright (c) Camilla Löwy <elmindreda@glfw.org>
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would
//    be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not
//    be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source
//    distribution.
//
//========================================================================
//! [code]

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <nanogui/nanogui.h>

#include "linmath.h"

#include <stdlib.h>
#include <stdio.h>

static const struct
{
    float x, y;
    float r, g, b;
} vertices[3] =
{
    { -0.6f, -0.4f, 1.f, 0.f, 0.f },
    {  0.6f, -0.4f, 0.f, 1.f, 0.f },
    {   0.f,  0.6f, 0.f, 0.f, 1.f }
};

static const char* vertex_shader_text =
"#version 110\n"
"uniform mat4 MVP;\n"
"attribute vec3 vCol;\n"
"attribute vec2 vPos;\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vPos, 0.0, 1.0);\n"
"    color = vCol;\n"
"}\n";

static const char* fragment_shader_text =
"#version 110\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_FragColor = vec4(color, 1.0);\n"
"}\n";

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

int main(void)
{
    GLFWwindow* windowGL2;
    GLuint vertex_buffer, vertex_shader, fragment_shader, program;
    GLint mvp_location, vpos_location, vcol_location;

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    windowGL2 = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
    if (!windowGL2)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    // setup OpenGL 2.0 window
    glfwSetKeyCallback(windowGL2, key_callback);

    glfwMakeContextCurrent(windowGL2);
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
    glfwSwapInterval(1);

    // NOTE: OpenGL error checks have been omitted for brevity

    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);

    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);

    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    mvp_location = glGetUniformLocation(program, "MVP");
    vpos_location = glGetAttribLocation(program, "vPos");
    vcol_location = glGetAttribLocation(program, "vCol");

    glEnableVertexAttribArray(vpos_location);
    glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE,
                          sizeof(vertices[0]), (void*) 0);
    glEnableVertexAttribArray(vcol_location);
    glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertices[0]), (void*) (sizeof(float) * 2));

    // Let's make nano gui!
    nanogui::init();

    nanogui::Screen *app = new nanogui::Screen({200, 400}, "NanoGUI Test", true, false, 8, 8, 24, 8, 0, 3, 3);

    nanogui::FormHelper *mainMenu = new nanogui::FormHelper(app);
    mainMenu->addWindow(Eigen::Vector2i(0, 0), "Main Menu");
    mainMenu->addGroup("Visualization");
    bool isTest;
    mainMenu->addVariable("Some checkbox", isTest);

    app->performLayout();

    app->drawAll();
    app->setVisible(true);

    while (!glfwWindowShouldClose(app->glfwWindow()) && !glfwWindowShouldClose(windowGL2))
    {
        // draw the OpenGL 2.0 window
        glfwMakeContextCurrent(windowGL2);
        float ratio;
        int width, height;
        mat4x4 m, p, mvp;

        glfwGetFramebufferSize(windowGL2, &width, &height);
        ratio = width / (float) height;

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        mat4x4_identity(m);
        mat4x4_rotate_Z(m, m, (float) glfwGetTime());
        mat4x4_ortho(p, -ratio, ratio, -1.f, 1.f, 1.f, -1.f);
        mat4x4_mul(mvp, p, m);

        glUseProgram(program);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, (const GLfloat*) mvp);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glfwSwapBuffers(windowGL2);

        // draw the nanogui
        glfwMakeContextCurrent(app->glfwWindow());

        glClearColor(0,0,255,1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        app->drawWidgets();

        glfwSwapBuffers(app->glfwWindow());
        glfwPollEvents();
    }

    glfwDestroyWindow(windowGL2);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

//! [code]


//////////////////////////////////////////////////////////////////////////

//#include <GUILib/GLApplication.h>

////#include <iostream>
////#include <nanogui/opengl.h>
////#include <GL/glu.h>

//int main() {
//    GLApplication app;
//    app.runMainLoop();

////    if (!glfwInit()) {
////        // An error occured
////        std::cout << "GLFW initialization failed\n";
////        exit(0);
////    }

////    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
////    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
////    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
//////    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

////    // Pointer to glfw window
////    GLFWwindow* glfwWindow;
////    glfwWindow = glfwCreateWindow(800, 600, "", NULL, NULL);


////    if (!glfwWindow) {
////        std::cout << "Could not initialize GLFW window\n";
////        glfwTerminate();
////        exit(0);
////    }
////    else
////    {
////        std::cout << "GLFW window: SUCCESS\n";

////    }

//}

////#include <nanogui/screen.h>
////#include <nanogui/window.h>
////#include <nanogui/layout.h>
////#include <nanogui/button.h>
////#include <nanogui/glutil.h>
////#include <nanogui/label.h>
////#include <nanogui/theme.h>
////#include <nanogui/formhelper.h>
////#include <nanogui/slider.h>

////#include <OptimizationLib/ObjectiveFunction.h>
////#include <RBSimLib/HingeJoint.h>

////using namespace std;
////using nanogui::Screen;
////using nanogui::Window;
////using nanogui::GroupLayout;
////using nanogui::Button;
////using nanogui::Vector2f;
////using nanogui::MatrixXu;
////using nanogui::MatrixXf;
////using nanogui::Label;

////class TestFunction : public ObjectiveFunction {
////public:
////    TestFunction() {}
////    ~TestFunction() {}

////    double alpha = 0.05;

////    double computeValue(const dVector& m) {
////        //Test case 1: Rosenbrock function
////        double c1 = 1 - m[0];
////        double c2 = m[1] - 0.5 * m[0] * m[0];
////        return alpha * (c1 * c1 + 10 * c2 * c2);
////    }
////};

////int main() {

////    // quick test of optimization lib
////    TestFunction testFunc;
////    dVector m(2);
////    m[0] = 1;
////    m[1] = 2;
////    double v = testFunc.computeValue(m);
////    std::cout << "value of test function = " << v << std::endl;

////    // quick test of RBSimLib
////    HingeJoint joint;

////    nanogui::init();

////    /**
////     * Create a screen, add a window.
////     * To the window add a label and a slider widget.
////     */

////    Screen app{{1024 / 2, 768 / 2}, "NanoGUI Test"};

////    Window window{&app, ""};
////    window.setPosition({15, 15});
////    window.setLayout(new GroupLayout(5, 5, 0, 0));

////    Label *l = new Label(&window,"MODULATION","sans-bold");
////    l->setFontSize(10);
////    nanogui::Slider *slider = new nanogui::Slider(&window);
////    slider->setValue(0.5f);
////    float modulation = 5.0f;
////    slider->setCallback([&modulation](float value) { modulation = value * 10.0f; });

////    // Do the layout calculations based on what was added to the GUI
////    app.performLayout();

////    app.drawAll();
////    app.setVisible(true);

////    /**
////     * 10: clear screen
////     * 20: set modulation value
////     * 30: draw using shader
////     * 40: draw GUI
////     * 50: goto 10
////     */
////    while (!glfwWindowShouldClose(app.glfwWindow()))
////    {
////        glClearColor(0,0,0,1);
////        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

////		app.drawWidgets();
////		//joint.drawAxes();

////        glfwSwapBuffers(app.glfwWindow());
////        glfwPollEvents();
////    }

////    nanogui::shutdown();
////    exit(EXIT_SUCCESS);
////}
