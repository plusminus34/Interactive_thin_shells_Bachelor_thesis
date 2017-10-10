#pragma once

#define __CSTROBOT_INDEX_CHECK_(ret) \
if (ID < 0 || (uint)ID >= nodes.size()) \
{ \
	Logger::consolePrint("INDEX OUT OF BOUND!"); \
	return (ret); \
}
#define __CSTROBOT_CENTER_CHECK_(ret) \
if (nodes[ID]->getType() != CENTER) \
{ \
	Logger::consolePrint("NOT A BODY!"); \
	return (ret); \
}
#define __CSTROBOT_BODYOFCENTER_CHECK_(ret) \
if (nodes[ID]->getParent() == NULL) \
{ \
	Logger::consolePrint("CANNOT FIND BODY BY THIS CENTER!"); \
	return (ret); \
}
#define __CSTROBOT_CENTER_CHECK_POINTER_(ret) \
if (node->getType() != CENTER) \
{ \
	Logger::consolePrint("NOT A BODY!"); \
	return (ret); \
}
#define __CSTROBOT_BODYOFCENTER_CHECK_POINTER_(ret) \
if (node->getParent() == NULL) \
{ \
	Logger::consolePrint("CANNOT FIND BODY BY THIS CENTER!"); \
	return (ret); \
}



#define __CSTROBOT_INDEX_CHECK_NO_PRINT_(ret) \
if (ID < 0 || (uint)ID >= nodes.size()) \
{ \
	return (ret); \
}
#define __CSTROBOT_CENTER_CHECK_NO_PRINT_(ret) \
if (nodes[ID]->getType() != CENTER) \
{ \
	return (ret); \
}
#define __CSTROBOT_BODYOFCENTER_CHECK_NO_PRINT_(ret) \
if (nodes[ID]->getParent() == NULL) \
{ \
	return (ret); \
}
#define __CSTROBOT_CENTER_CHECK_POINTER_NO_PRINT_(ret) \
if (node->getType() != CENTER) \
{ \
	return (ret); \
}
#define __CSTROBOT_BODYOFCENTER_CHECK_POINTER_NO_PRINT_(ret) \
if (node->getParent() == NULL) \
{ \
	return (ret); \
}