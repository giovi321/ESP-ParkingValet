#pragma once
bool  clfAvailable();              // true only when a real model is embedded
float clfScore(const float* feat); // occupied probability [0,1], or -1 if unavailable
