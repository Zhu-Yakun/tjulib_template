#include "tjulib/velPID.hpp"
#include "vex.h"
#include<cmath>

//sgn
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
} 

velPID::velPID(float Kp, float Kd, float Ki){
  m_Kp = Kp;
  m_Kd = Kd;
  m_Ki = Ki;
}

float velPID::calculate(float wantedRPM, float currentRPM) {
  m_Error = wantedRPM - currentRPM;
  m_integral += m_Error;
  m_derivative = (m_Error - m_lastError);
  m_lastError = m_Error;

  float finalPower = (m_Error * m_Kp) + (m_derivative * m_Kd) + (m_integral * m_Ki);

  if(std::abs(finalPower) > 200) { finalPower = sgn(finalPower) * 200; }

  return finalPower;
}

void velPID::setGains(float Kp, float Kd, float Ki) {
  m_Kp = Kp;
  m_Kd = Kd;
  m_Ki = Ki;

}

float velPID::getError() { return m_Error; }