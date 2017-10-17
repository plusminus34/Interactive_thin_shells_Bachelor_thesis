#include <iostream>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/glutil.h>
#include <nanogui/label.h>
#include <nanogui/theme.h>
#include <nanogui/formhelper.h>
#include <nanogui/slider.h>

//#include <OptimizationLib/ObjectiveFunction.h>
//#include <RBSimLib/HingeJoint.h>

using namespace std;
using nanogui::Screen;
using nanogui::Window;
using nanogui::GroupLayout;
using nanogui::Button;
using nanogui::Vector2f;
using nanogui::MatrixXu;
using nanogui::MatrixXf;
using nanogui::Label;

//class TestFunction : public ObjectiveFunction {
//public:
//	TestFunction() {}
//	~TestFunction() {}

//	double alpha = 0.05;

//	double computeValue(const dVector& m) {
//		//Test case 1: Rosenbrock function
//		double c1 = 1 - m[0];
//		double c2 = m[1] - 0.5 * m[0] * m[0];
//		return alpha * (c1 * c1 + 10 * c2 * c2);
//	}
//};

int main() {

//	// quick test of optimization lib
//	TestFunction testFunc;
//	dVector m(2);
//	m[0] = 1;
//	m[1] = 2;
//	double v = testFunc.computeValue(m);
//	std::cout << "value of test function = " << v << std::endl;

//	// quick test of RBSimLib
//	HingeJoint joint;

    nanogui::init();

    /**
     * Create a screen, add a window.
     * To the window add a label and a slider widget.
     */

    Screen app{{1024 / 2, 768 / 2}, "NanoGUI Test"};

    Window window{&app, ""};
    window.setPosition({15, 15});
    window.setLayout(new GroupLayout(5, 5, 0, 0));

    Label *l = new Label(&window,"MODULATION","sans-bold");
    l->setFontSize(10);
    nanogui::Slider *slider = new nanogui::Slider(&window);
    slider->setValue(0.5f);
    float modulation = 5.0f;
    slider->setCallback([&modulation](float value) { modulation = value * 10.0f; });

    // Do the layout calculations based on what was added to the GUI
    app.performLayout();

    app.drawAll();
    app.setVisible(true);

    /**
     * 10: clear screen
     * 20: set modulation value
     * 30: draw using shader
     * 40: draw GUI
     * 50: goto 10
     */
    while (!glfwWindowShouldClose(app.glfwWindow()))
    {
        glClearColor(0,0,0,1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		app.drawWidgets();
		//joint.drawAxes();

        glfwSwapBuffers(app.glfwWindow());
        glfwPollEvents();
    }

    nanogui::shutdown();
    exit(EXIT_SUCCESS);
}
