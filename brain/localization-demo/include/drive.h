// PROS manual mecanum drive. No localization or path following dependencies.
#pragma once

namespace drive
{

bool initialize(int imu_port);
void update(); // read master controller and command the four motor pairs
void stop();   // also require centered sticks before driving resumes
const char* status();

} // namespace drive
