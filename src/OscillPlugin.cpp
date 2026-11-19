/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
* libscopehal's InitializePlugins() dlopen()s every file in ~/.scopehal/plugins (and /usr/lib/scopehal/plugins,       *
* /usr/local/lib/scopehal/plugins, the executable's directory) and calls PluginInit().                                *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "OscillOscilloscope.h"

extern "C" __attribute__((visibility("default"))) void PluginInit()
{
	AddDriverClass(OscillOscilloscope);
	LogDebug("Oscill DSO1 plugin loaded (driver \"oscill\")\n");
}
