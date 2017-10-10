
#pragma once

#ifndef	ASSERT

#ifdef _DEBUG

#include <assert.h>

#include <iostream>
#include <sstream>

#define ASSERT(e, msg)	if (! (e)) { \
	std::ostringstream os_;    \
   os_ << (msg) << std::endl; \
os_ << "File: " << __FILE__ << std::endl; \
	os_ << "Line: " << __LINE__ << std::endl; \
	 Logger::consolePrint( os_.str().c_str() ); \
	 assert( (e) ); }
#else
	#define ASSERT(e, msg) (void)(e); (void)(msg)
#endif
#endif


