#pragma once

class velPID {
private:
  float m_Kp = 0;
  float m_Kd = 0;
  float m_Ki = 0;

  float m_Error = 0;
  float m_lastError = 0;
  float m_derivative = 0;
  float m_integral = 0;

public:
  velPID(float Kp, float Kd, float Ki);
  float calculate(float wantedRPM, float currentRPM);
  void setGains(float Kp, float Kd, float Ki); 
  float getError();
};