#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include <Arduino.h>

bool isLiveControlKey(const String& key);
bool isLiveRemoteTextCommand(const String& cmd);
String processKeyValueCommand(const String& keyIn, const String& valueIn, const String& via);
String processRemoteCommand(const String& command);

#endif
