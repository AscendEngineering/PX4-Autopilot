

#include "throw_example.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState ThrowApp::appState;

int ThrowApp::main()
{
	appState.setRunning(true);

	int i = 0;

	while (!appState.exitRequested() && i < 5) {
		px4_sleep(2);

		printf("  Doing work...\n");
		++i;
	}

	return 0;
}
