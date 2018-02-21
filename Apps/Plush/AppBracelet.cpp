#include <GUILib/GLUtils.h>
#include "AppBracelet.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <Marshmallow/marshmallow_star.h>
#include <GUILib/GLTrackingCamera.h>
#include <MathLib/MathLib.h>

AppBracelet::AppBracelet() {
    setWindowTitle("AppBracelet");
	// ((GLTrackingCamera *)camera)->rotAboutRightAxis = .612;
	// ((GLTrackingCamera *)camera)->rotAboutUpAxis = .37;
	((GLTrackingCamera *)camera)->camDistance = -1.16;
	((GLTrackingCamera *)camera)->camTarget = P3D(0., .16, 0.);

	showConsole = false;
	appIsRunning = true;
	showGroundPlane = false;
	showDesignEnvironmentBox = false;

	desiredFrameRate = 19;

    TwAddSeparator(mainMenuBar, "", "");
	TwAddVarRW(mainMenuBar, "flush all buffers",        TW_TYPE_BOOLCPP, &FLUSH_ALL_BUFFERS,        "");
	TwAddVarRW(mainMenuBar, "initiate all connections", TW_TYPE_BOOLCPP, &INITIATE_ALL_CONNECTIONS, "");
	TwAddVarRW(mainMenuBar, "query all positions",      TW_TYPE_BOOLCPP, &QUERY_ALL_POSITIONS,      "");
	TwAddVarRW(mainMenuBar, "set all homes",            TW_TYPE_BOOLCPP, &SET_ALL_HOMES,            "");
	TwAddVarRW(mainMenuBar, "goto all homes",           TW_TYPE_BOOLCPP, &GOTO_ALL_HOMES,           "");
	TwAddVarRW(mainMenuBar, "close all connections",    TW_TYPE_BOOLCPP, &CLOSE_ALL_CONNECTIONS,    "");
	TwAddVarRW(mainMenuBar, "send splines",             TW_TYPE_BOOLCPP, &SEND_SPLINES,             "");

    system("cls");
	cout << "AppBracelet-----------------------------------------------------------------------" << endl;

	// // build spline
	cout << "FORNOW: Building splines." << endl;
	// <-0.25, (0.0, 1.0)>
	// <+0.00, (0.0, 0.0)>
	// <+0.25, (1.0, 0.0)>
	// <+0.50, (0.0, 0.0)>
	// <+0.75, (0.0, 1.0)>
	// <+1.00, (0.0, 0.0)>
	// <+1.25, (1.0, 0.0)>
	double HIGH = 1.00;
	double SLAK = -.25*HIGH;
	double REST = 0.15;
	double X_arr[] = {-0.25, +0.00, +0.25, +0.50, +0.75, +1.00, +1.25};
	double L_arr[] = { SLAK,  REST,  HIGH,  REST,  SLAK,  REST,  HIGH};
	double R_arr[] = { HIGH,  REST,  SLAK,  REST,  HIGH,  REST,  SLAK};


	vector<double> X;
	vector<double> L;
	vector<double> R;
	// ---
	for (int i = 0; i < 7; ++i) {
		X.push_back(X_arr[i]);
		L.push_back(L_arr[i]);
		R.push_back(R_arr[i]);
	}
	spline_L.set_points(X, L);
	spline_R.set_points(X, R);

	cout << "FORNOW: Built splines." << endl;
}

AppBracelet::~AppBracelet(void) {
}

////////////////////////////////////////////////////////////////////////////////

void AppBracelet::process() {

	if (SEND_SPLINES) {
		frame++;
		send_splines();
	}
 
	if (FLUSH_ALL_BUFFERS) {
		FLUSH_ALL_BUFFERS = false;
		if (!flush_buffer(hCommL)) { cout_failure("Failed to flush hCommL.\n"); }
		if (!flush_buffer(hCommR)) { cout_failure("Failed to flush hCommR.\n"); }
	}

	if (INITIATE_ALL_CONNECTIONS && !CONNECTIONS_INITIATED) {
		INITIATE_ALL_CONNECTIONS = false;
		CONNECTIONS_INITIATED = true;
		if (!initiate_connection(hCommL, COM_PORT_L)) { cout_failure("Failed to initiate hCommL.\n"); }
		if (!initiate_connection(hCommR, COM_PORT_R)) { cout_failure("Failed to initiate hCommR.\n"); }
	}

	if (QUERY_ALL_POSITIONS) {
		QUERY_ALL_POSITIONS = false;
		if (!flush_buffer(hCommL)) { cout_failure("Failed to flush hCommL.\n"); }
		if (!flush_buffer(hCommR)) { cout_failure("Failed to flush hCommR.\n"); }

		if (!query_position(hCommL)) { cout_failure("Failed to query position on hCommL.\n"); }
		if (!query_position(hCommR)) { cout_failure("Failed to query position on hCommR.\n"); }
	}

	if (SET_ALL_HOMES) {
		SET_ALL_HOMES = false;
		if (!set_home(hCommL)) { cout_failure("Failed to set home on hCommL.\n"); }
		if (!set_home(hCommR)) { cout_failure("Failed to set home on hCommR.\n"); }
	}

	if (GOTO_ALL_HOMES) {
		GOTO_ALL_HOMES = false;
		if (!goto_home(hCommL)) { cout_failure("Failed to goto home on hCommL.\n"); }
		if (!goto_home(hCommR)) { cout_failure("Failed to goto home on hCommR.\n"); }
	}

	if (CLOSE_ALL_CONNECTIONS) {
		CLOSE_ALL_CONNECTIONS = false;
		if (!close_connection(hCommL)) { cout_failure("Failed to close hCommL.\n"); }
		if (!close_connection(hCommR)) { cout_failure("Failed to close hCommR.\n"); }
	}

}

void AppBracelet::drawScene() {

	// cout << ((GLTrackingCamera *)camera)->camDistance << endl;
	// cout << ((GLTrackingCamera *)camera)->camTarget << endl;

	dynamic_cast<GLTrackingCamera *>(camera)->rotAboutRightAxis = 0;
	dynamic_cast<GLTrackingCamera *>(camera)->rotAboutUpAxis = 0;

	handle_bern_errors_and_warnings();
	draw_arrow2d(P3D(), 1, 1, 0.);
}


////////////////////////////////////////////////////////////////////////////////
// master  /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool AppBracelet::set_position(HANDLE &hComm, int position) {
	// cout << "Setting position..." << endl;

	std::string pos = std::to_string(position);
	std::string write_buffer_str = "LA" + pos + "\r\nM\r\n";
	char write_buffer[1024];
	strcpy(write_buffer, write_buffer_str.c_str());
	unsigned int magic = 2U + pos.length() + 5U;
	write_buffer[magic] = 0;

	if (!WriteFile(hComm, write_buffer, (magic + 1U), &written, NULL)) {
		cout << write_buffer << endl;
		CoutLastError();
		return false;
	}

	// cout_success("Successfully set position.\n");
	return true;
}

bool AppBracelet::send_splines() {

	int F = 200;

	double t_raw = double(frame) / double(F);
	double t = t_raw - long(t_raw);
	double L_t = spline_L(t);
	double R_t = spline_R(t);
	// cout << std::setprecision(2) << t << ": (" << L_t << ", " << R_t << ")" << endl;

	int L_position = int(L_t * double(MAX_POS_L));
	int R_position = int(R_t * double(MAX_POS_R));
 
	if (L_position > MAX_POS_L) { cout << "WHAT L!" << endl; L_position = MAX_POS_L; }
	if (R_position > MAX_POS_R) { cout << "WHAT R!" << endl; R_position = MAX_POS_R; }
	// if (L_position < 0) { L_position = 0; } // FORNOW
	// if (R_position < 0) { R_position = 0; } // FORNOW

	// cout << ": (" << L_position << ", " << R_position << ")" << endl;
	set_position(hCommL, L_position);
	set_position(hCommR, R_position);

	return true; 
}


////////////////////////////////////////////////////////////////////////////////
// serial  /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const auto s2ws = [](const std::string& s) -> LPCWSTR {
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0); 
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r.c_str();
};

//Returns the last Win32 error, in string format. Returns an empty string if there is no error.
std::string GetLastErrorAsString() {
    //Get the error message, if any.
    DWORD errorMessageID = ::GetLastError();
    if(errorMessageID == 0)
        return std::string(); //No error message has been recorded

    LPSTR messageBuffer = nullptr;
    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                 NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

    std::string message(messageBuffer, size);

    //Free the buffer.
    LocalFree(messageBuffer);

    return message;
}

////////////////////////////////////////////////////////////////////////////////

void AppBracelet::CoutLastError() { 
	cout_warning(GetLastErrorAsString());
}

void AppBracelet::cout_failure(std::string msg) {
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, 12);
	cout << msg;
	SetConsoleTextAttribute(hConsole, 15);
}

void AppBracelet::cout_success(std::string msg) {
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, 10);
	cout << msg;
	SetConsoleTextAttribute(hConsole, 15);
}

void AppBracelet::cout_warning(std::string msg) {
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, 14);
	cout << msg;
	SetConsoleTextAttribute(hConsole, 15);
}

////////////////////////////////////////////////////////////////////////////////

bool AppBracelet::initiate_connection(HANDLE &hComm, int COM_PORT) {
	cout << "Initiating connection on COM" << COM_PORT << "..." << endl;

	cout << "--- Opening port..." << endl;
	std::wstring comPrefix = L"\\\\.\\COM";
	std::wostringstream tmp;
	tmp << COM_PORT;
	const std::wstring comSuffix(tmp.str()); 
	std::wstring port_spec = comPrefix + comSuffix;
	// https://support.microsoft.com/en-us/help/115831/howto-specify-serial-ports-larger-than-com9
	hComm = CreateFileW(
		port_spec.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (hComm == INVALID_HANDLE_VALUE) {
		return false;
	}

	cout << "--- Applying settings..." << endl;
	std::wstring dcb_spec = L"9600, n, 8, 1";
	DCB dcb;
	FillMemory(&dcb, sizeof(dcb), 0);
	dcb.DCBlength = sizeof(dcb);
	if (!BuildCommDCBW(dcb_spec.c_str(), &dcb)) {
		return false;
	}
	if (!SetCommState(hComm, &dcb)) {
		return false;
	}

	cout << "--- Specifying timeouts..." << endl;
    COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 20; 
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 100;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 100;
	if (!SetCommTimeouts(hComm, &timeouts)) {
		return false;
	}

	cout_success("Successfully initiated connection.\n");
	return true;
}

bool AppBracelet::flush_buffer(HANDLE &hComm) {
	cout << "Flushing buffer..." << endl;

	char read_buffer[1024];
	DWORD bytesRead = 0;
	if (!ReadFile(hComm, read_buffer, sizeof(read_buffer), &bytesRead, NULL)) {
		CoutLastError();
		return false;
	}

	for (int i = 0; i < (int)bytesRead; ++i) {
		cout << read_buffer[i];
	}

	cout_success("Successfully flushed buffer.\n");
	return true;
}

bool AppBracelet::query_position(HANDLE &hComm) {
	cout << "Querying position." << endl;

	char write_buffer[] = "POS\r\n";
	if (!WriteFile(hComm, write_buffer, sizeof(write_buffer), &written, NULL)) {
		CoutLastError();
		return false; 
	}
	char read_buffer[128];
	DWORD bytesRead = 0;
	if (!ReadFile(hComm, read_buffer, sizeof(read_buffer), &bytesRead, NULL)) {
		CoutLastError();
		return false;
	}

	read_buffer[(int)bytesRead - 2] = 0; // Killing off carriage return
	cout << "Position: " << read_buffer << endl;

	cout_success("Successfully queried position.\n");
	return true;
}

bool AppBracelet::set_home(HANDLE &hComm) {
	cout << "Setting home..." << endl;

	char write_buffer[] = "HO\r\n";

	if (!WriteFile(hComm, write_buffer, sizeof(write_buffer), &written, NULL)) {
		CoutLastError();
		return false;
	}

	cout_success("Successfully set home.\n");
	return true;
}

bool AppBracelet::goto_home(HANDLE &hComm) {
	cout << "Going home..." << endl;

	char write_buffer[] = "LA0\r\nM\r\n";

	if (!WriteFile(hComm, write_buffer, sizeof(write_buffer), &written, NULL)) {
		CoutLastError();
		return false;
	}

	cout_success("Successfully set home.\n");
	return true;
}

bool AppBracelet::close_connection(HANDLE &hComm) {
	cout << "Closing connection..." << endl;

	if (hComm == INVALID_HANDLE_VALUE) {
		cout_warning("Handle invalid.\n");
		return false;
	}
	CloseHandle(hComm);
	hComm = INVALID_HANDLE_VALUE;

	cout_success("Successfully closed connection.\n");
	return true;
}


////////////////////////////////////////////////////////////////////////////////
// glfw ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool AppBracelet::onMouseMoveEvent(double xPos, double yPos) {

	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) {
		return true;
	}
	return false;
}

bool AppBracelet::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) {
		return true;
	}
	return false;
}



bool AppBracelet::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) {
		return true;
	}

    return false;
}

bool AppBracelet::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) {
		return true;
	}

    return false;
}

bool AppBracelet::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) {
		return true;
	}

    return false;
}

////////////////////////////////////////////////////////////////////////////////

void AppBracelet::drawAuxiliarySceneInfo() {
}

void AppBracelet::restart() {
}

bool AppBracelet::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) {
		return true;
	}
    return false;
}
