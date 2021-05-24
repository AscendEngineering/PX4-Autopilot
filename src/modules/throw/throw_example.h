
#pragma once

#include <px4_platform_common/app.h>

class ThrowApp
{
public:
	ThrowApp() {}

	~ThrowApp() {}

	int main();

	static px4::AppState appState; /* track requests to terminate app */
};
