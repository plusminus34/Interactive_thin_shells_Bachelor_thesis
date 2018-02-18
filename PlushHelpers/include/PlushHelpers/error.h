#pragma once

#include <string>
 
extern bool BERN_ERROR;
extern bool BERN_ERROR_REPORTING;
extern const int N_COOLDOWN;
extern int BERN_ERROR_COOLDOWN;

void error(const std::string &msg);
