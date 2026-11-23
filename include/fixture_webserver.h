#pragma once
#include <ESPAsyncWebServer.h>

// Unified fixture webserver interface — each fixture implements both of these
// in fixtures/<name>/webserver.cpp. The SPA in data/ renders the UI client-side,
// so the firmware only needs the JSON save path and fixture-specific routes.

// Handle fixture-specific form fields from POST /save (legacy form path) and
// from POST /api/config (SPA path — body fields go through here).
void handleFixtureSaveParams(AsyncWebServerRequest* request, bool& needsRestart);

#ifdef RAVLIGHT_ORION_COMPACT_DMX_POOL
#include <ArduinoJson.h>
bool reserveOrionFormDmxPatch(AsyncWebServerRequest* request, uint16_t motorUniverse);
bool reserveOrionJsonDmxPatch(JsonObjectConst fixture, uint16_t motorUniverse);
#endif

// Register fixture-specific HTTP routes (e.g. /highlight, /home, /sgcal).
void registerFixtureRoutes(AsyncWebServer& server);
