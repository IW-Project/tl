#pragma once

class pulse_sum_cache
{
private:
  float m_pulse_sum;

public:
  inline pulse_sum_cache()
    : m_pulse_sum( 0.0f )
  {
  }
  inline ~pulse_sum_cache() {}

  inline void zero_pulse_sum() { m_pulse_sum = 0; }
  inline void set_pulse_sum( const float pulse_sum ) { m_pulse_sum = pulse_sum; }
  inline float get_pulse_sum() const { return m_pulse_sum; }
};
