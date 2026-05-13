#pragma once

class rigid_body;
class pulse_sum_constraint_solver;

class rigid_body_constraint
{
  friend class rbcint;

protected:
  rigid_body *b1;
  rigid_body *b2;
  rigid_body_constraint *m_next;

public:
  rigid_body *get_b1() const { return b1; }

  rigid_body *get_b2() const { return b2; }

  virtual void zero_pulse_sums() {}
  virtual void setup_constraint( pulse_sum_constraint_solver *solver, const float delta_t ) {}
  virtual void epilog_vel_constraint( const float delta_t ) {}
  virtual void outer_prolog_update( const float delta_t ) {}
  virtual void inner_update( const float delta_t ) {}
  virtual void outer_epilog_update( const float delta_t ) {}
};